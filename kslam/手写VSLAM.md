# 手写VSLAM

参看高翔视觉SLAM十四讲，手写一个直接法（特征点法）的视觉SLAM程序。程序=数据结构+算法，我们首先来看看数据结构。

# 01 数据结构

## kslam/frame.h

我们手写的Frame含有关键帧id，位子，图像以及特征点。其中`Pose`会被后端同事设置或访问，所以定义`Pose`的`Set`和`Get`函数。在函数内加锁

```cpp
struct Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;           // id of this frame
    unsigned long keyframe_id_ = 0;  // id of key frame
    bool is_keyframe_ = false;       // 是否为关键帧
    double time_stamp_;              // 时间戳，暂不使用
    SE3 pose_;                       // Tcw 形式Pose
    std::mutex pose_mutex_;          // Pose数据锁
    cv::Mat left_img_, right_img_;   // stereo images

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

   public:  // data members
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
          const Mat &right);

    // set and get pose, thread safe
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /// 设置关键帧并分配并键帧id
    void SetKeyFrame();

    /// 工厂构建模式，分配id 
    static std::shared_ptr<Frame> CreateFrame();
};
```

### EIGEN_MAKE_ALIGNED

这里注意，使用`Eigen`库的时候有时候会报错

```bash
my_program: path/to/eigen/Eigen/src/Core/DenseStorage.h:44: Eigen::internal::matrix_array<T, Size, MatrixOptions, Align>::internal::matrix_array() [with T = double, int Size = 2, int MatrixOptions = 2, bool Align = true]: Assertion `(reinterpret_cast(array) & (sizemask)) == 0 && "this assertion is explained here: http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html READ THIS WEB PAGE !!! ****"' failed.
```

这时候就需要加入一个宏`EIGEN_MAKE_ALIGNED_OPERATOR_NEW`。这个问题的根本原因是Eigen为了提高运算速度，采取了128位内存对齐，以让编译器进行向量化优化。而如果自己的new就不会有内存对已，因此需要加上一个宏，重新实现内存对齐的new.

### SHARED_PTR 智能指针

主要需要注意二次析构的问题

>为了解决C++内存泄漏的问题，C++11引入了智能指针（Smart Pointer）。
>
>　　智能指针的原理是，接受一个申请好的内存地址，构造一个保存在栈上的智能指针对象，当程序退出栈的作用域范围后，由于栈上的变量自动被销毁，智能指针内部保存的内存也就被释放掉了（除非将智能指针保存起来）。
>
>　　C++11提供了三种智能指针：std::shared_ptr, std::unique_ptr, std::weak_ptr，使用时需添加头文件<memory>。
>
>　　shared_ptr使用引用计数，每一个shared_ptr的拷贝都指向相同的内存。每使用他一次，内部的引用计数加1，每析构一次，内部的引用计数减1，减为0时，删除所指向的堆内存。shared_ptr内部的引用计数是安全的，但是对象的读取需要加锁。

!　注意，不能将一个原始指针直接赋值给一个智能指针，如下所示，原因是一个是类，一个是指针。

```cpp
std::shared_ptr<int> p4(new int(5));
int *pInt = p4.get();
```

* 不要用一个原始指针初始化多个shared_ptr，原因在于，会造成二次销毁，如下所示：

  ```cpp
      int *p5 = new int;
      std::shared_ptr<int> p6(p5);
      std::shared_ptr<int> p7(p5);// logic error
  ```

* 不要在函数实参中创建shared_ptr。因为C++的函数参数的计算顺序在不同的编译器下是不同的。正确的做法是先创建好，然后再传入。

  ```cpp
  function(shared_ptr<int>(new int), g());
  ```

* 禁止通过shared_from_this()返回this指针，这样做可能也会造成**二次析构**。

* 避免循环引用。智能指针最大的一个陷阱是循环引用，循环引用会导致内存泄漏。解决方法是AStruct或BStruct改为weak_ptr。

### std::mutex

> Mutex 又称互斥量，C++ 11中与 Mutex 相关的类（包括锁类型）和函数都声明在 <mutex> 头文件中，所以如果你需要使用 std::mutex，就必须包含 <mutex> 头文件。

#### Mutex 系列类(四种)

- std::mutex，最基本的 Mutex 类。
- std::recursive_mutex，递归 Mutex 类。
- std::time_mutex，定时 Mutex 类。
- std::recursive_timed_mutex，定时递归 Mutex 类。

#### Lock 类（两种）

- std::lock_guard，与 Mutex RAII 相关，方便线程对互斥量上锁。
- std::unique_lock，与 Mutex RAII 相关，方便线程对互斥量上锁，但提供了更好的上锁和解锁控制。

#### 其他类型

- std::once_flag
- std::adopt_lock_t
- std::defer_lock_t
- std::try_to_lock_t

#### 函数

- std::try_lock，尝试同时对多个互斥量上锁。
- std::lock，可以同时对多个互斥量上锁。
- std::call_once，如果多个线程需要同时调用某个函数，call_once 可以保证多个线程对该函数只调用一次。

### std::mutex 介绍

下面以 std::mutex 为例介绍 C++11 中的互斥量用法。

std::mutex 是C++11 中最基本的互斥量，std::mutex 对象提供了独占所有权的特性——即不支持递归地对 std::mutex 对象上锁，而 std::recursive_lock 则可以递归地对互斥量对象上锁。

#### std::mutex 的成员函数

- 构造函数，std::mutex不允许拷贝构造，也不允许 move 拷贝，最初产生的 mutex 对象是处于 unlocked 状态的。
- lock()，调用线程将锁住该互斥量。线程调用该函数会发生下面 3 种情况：(1). 如果该互斥量当前没有被锁住，则调用线程将该互斥量锁住，直到调用 unlock之前，该线程一直拥有该锁。(2). 如果当前互斥量被其他线程锁住，则当前的调用线程被阻塞住。(3). 如果当前互斥量被当前调用线程锁住，则会产生死锁(deadlock)。
- unlock()， 解锁，释放对互斥量的所有权。
- try_lock()，尝试锁住互斥量，如果互斥量被其他线程占有，则当前线程也不会被阻塞。线程调用该函数也会出现下面 3 种情况，(1). 如果当前互斥量没有被其他线程占有，则该线程锁住互斥量，直到该线程调用 unlock 释放互斥量。(2). 如果当前互斥量被其他线程锁住，则当前调用线程返回 false，而并不会被阻塞掉。(3). 如果当前互斥量被当前调用线程锁住，则会产生死锁(deadlock)。

```cpp
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex

volatile int counter(0); // non-atomic counter
std::mutex mtx;           // locks access to counter

void attempt_10k_increases() {
    for (int i=0; i<10000; ++i) {
        if (mtx.try_lock()) {   // only increase if currently not locked:
            ++counter;
            mtx.unlock();
        }
    }
}

int main (int argc, const char* argv[]) {
    std::thread threads[10];
    for (int i=0; i<10; ++i)
        threads[i] = std::thread(attempt_10k_increases);

    for (auto& th : threads) th.join();
    std::cout << counter << " successful increases of the counter.\n";

    return 0;
}
```

### C++ thread 与并发

```cpp
做法1：使用函数作为线程入口
void function_1() {
    std::cout << "hello world " << std::endl;
}
int main(){
    std::thread myobj(function_1);//创建了线程，一旦创建，线程就开始执行
    myobj.join();//主线程阻塞到这里等待function_1执行完毕，当子线程执行完毕，主线程就继续执行
}
```



## kslam/camera.h

盖头文件主要是用于保存和设置相机参数，目前我们只针对针孔相机

#### TODO 扭曲率

```cpp

namespace myslam {

// Pinhole stereo camera model
class Camera {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
           baseline_ = 0;  // Camera intrinsics
    SE3 pose_;             // extrinsic, from stereo camera to single camera
    SE3 pose_inv_;         // inverse of extrinsics

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline,
           const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
        pose_inv_ = pose_.inverse();
    }

    SE3 pose() const { return pose_; }

    // return intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    // coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
};

}  // namespace kslam
```



## kslam/feature.h

接下来是保存特征点的`feature.h`,feature类最主要的信息是自身的2D位置，此外`is_outlier`为异常点的标志位。

```cpp
namespace kslam {

struct Frame;
struct MapPoint;

/**
 * 2D 特征点
 * 在三角化之后会被关联一个地图点
 */
struct Feature {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;         // 持有该feature的frame
    cv::KeyPoint position_;              // 2D提取位置
    std::weak_ptr<MapPoint> map_point_;  // 关联地图点

    bool is_outlier_ = false;       // 是否为异常点
    bool is_on_left_image_ = true;  // 标识是否提在左图，false为右图

   public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
}  // namespace kslam

#endif 
```



## kslam/mappoint.h

接下来是路标点，Mappoint最主要的信息是他的3D位置，即`pose_`变量，同样 需要对他进行上锁

```cpp
namespace kslam {
    struct Frame;
    struct Feature;
/**
 * 路标点类
 * 特征点在三角化之后形成路标点
 */
    struct MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long id_ = 0;  // ID
        bool is_outlier_ = false;
        Vec3 pos_ = Vec3::Zero();  // Position in world
        std::mutex data_mutex_;
        int observed_times_ = 0;  // being observed by feature matching algo.
        std::list<std::weak_ptr<Feature>> observations_;

        MapPoint() {}

        MapPoint(long id, Vec3 position);

        Vec3 Pos() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        void SetPos(const Vec3 &pos) {
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        };

        void AddObservation(std::shared_ptr<Feature> feature) {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push_back(feature);
            observed_times_++;
        }

        void RemoveObservation(std::shared_ptr<Feature> feat);

        std::list<std::weak_ptr<Feature>> GetObs() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observations_;
        }

        // factory function
        static MapPoint::Ptr CreateNewMappoint();
    };
}  // namespace kslam
```



## kslam/map.h

在框架中，我们要让这些数据接欧持有这些Framehe和Mappoint对象，因此我们还需要定义一个地图类。地图以散列的形式记录了所有关键帧和对应的路标点，同事维护一个被激活的关键帧地图.

```cpp

namespace kslam {

/**
 * @brief 地图
 * 和地图的交互：前端调用InsertKeyframe和InsertMapPoint插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等等
 */
    class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        Map() {}

        /// 增加一个关键帧
        void InsertKeyFrame(Frame::Ptr frame);
        /// 增加一个地图顶点
        void InsertMapPoint(MapPoint::Ptr map_point);

        /// 获取所有地图点
        LandmarksType GetAllMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }
        /// 获取所有关键帧
        KeyframesType GetAllKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        /// 获取激活地图点
        LandmarksType GetActiveMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        /// 获取激活关键帧
        KeyframesType GetActiveKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }

        /// 清理map中观测数量为零的点
        void CleanMap();

    private:
        // 将旧的关键帧置为不活跃状态
        void RemoveOldKeyframe();

        std::mutex data_mutex_;
        LandmarksType landmarks_;         // all landmarks
        LandmarksType active_landmarks_;  // active landmarks
        KeyframesType keyframes_;         // all key-frames
        KeyframesType active_keyframes_;  // all key-frames

        Frame::Ptr current_frame_ = nullptr;

        // settings
        int num_active_keyframes_ = 7;  // 激活的关键帧数量
    };
}  // namespace kslam

```

至此我们就完成了所有的数据结构，接下来我们继续完成slam前端。



# 02 前端

在定义了基本数据结构之后，我们来考虑前端的功能，首先前端需要根据图像确定该帧的位姿。不过实际实现的时候还存在一些不同的方法。例如我们应该如何使用右目的图像呢，是每一帧都讲左右目进行比较还是比较其中一个呢？三角化的时候我们是用左右目三角化还是前后帧三角化呢?

在解决这些问题之前，我们先来看看前端的处理逻辑

* 前端有：初始化，正常追踪，追踪丢失等三个状态。
* 在初始化状态中，根据左右目之间的光流匹配可以找到三角化的地图点，建立初始地图。
* 追踪阶段中，前端计算上一帧的特诊点到这这一帧的光流，根据光溜匹配的结果来计算图像位姿。
* 如果追踪到点较少，就判定当前帧为关键帧。对于关键帧做一下几件事：
  * 提取新的特征点。
  * 找到这些点在右图中的对应点，利用三角化建立路标点。
  * 将新的关键帧和路标点加入地图，并且出发一次后端优化。
* 如果丢失，就充值前端系统们进行初始化。

> 也就是说，后端只在前端难顶的时候。

首先我们来看看调用顺序 

```cpp
vo->Run();
void VisualOdometry::Run() {
    while (1) {
        LOG(INFO) << "VO is running";
        if (Step() == false) {
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}
bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

Frame::Ptr Dataset::NextFrame() {
	// 数据结构中读取新frame的地方
}
```

注意到在Step里面，通过`AddFrame`函数来调用前端，对`NextFrame`中获得的新图像进行 处理，那么我们就开始详细看看前端如何处理的

```cpp
bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }
    last_frame_ = current_frame_;
    return true;
}
```

可以看到`AddFrame`就是一个状态机，在状态较差的时候`Track()`,在丢失的时候`Reset`，并且我们发现他是没有对于GOOD状态进行处理的，一直是假装状态有问题。我们继续来看。

这里要注意前端的构造函数

```cpp
Frontend::Frontend() {
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}
```

我们可以看出在KSLAM中采用的特征点GFTTDetecor

该方法是基于shi-tomas角点检测变化而来的一种特征提取方法，OpenCV创建该检测器的API与

我们来看看`Track（）`

```cpp
bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

```

注意这里的`relativ_motion_`是上一次计算出来的。

第二个环节我们就执行到了`TrackLastFrame`

对上个环节进行LK光流追踪，

```cpp
int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image
    // 创建
    std::vector<cv::Point2f> kps_last, kps_current;
    /*这里的kp是上一帧中所有特征的成员，是一个vector的元素. feature中包含自己的2D位置，包含锁*/
    for (auto &kp : last_frame_->features_left_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            // px是左边相机世界坐标到像素坐标的投影
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}
```

```cpp
world2pixel(mp->pos_, current_frame_->Pose())
```

我们着重来看OpenCV的函数

```cpp
void cv::calcOpticalFlowPyrLK	(	
InputArray 	prevImg,
InputArray 	nextImg,
InputArray 	prevPts,
InputOutputArray 	nextPts,
OutputArray 	status,
OutputArray 	err,
Size 	winSize = Size(21, 21),
int 	maxLevel = 3,
TermCriteria 	criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
int 	flags = 0,
double 	minEigThreshold = 1e-4 
)	

```

使用具有金字塔的迭代Lucas-Kanade方法计算稀疏特征集的光流。
参数：

* prevImg ：buildOpticalFlowPyramid构造的第一个8位输入图像或金字塔。
* nextImg ：与prevImg相同大小和相同类型的第二个输入图像或金字塔
* prevPts ：需要找到流的2D点的矢量(vector of 2D points for which the flow needs to be found;);点坐标须是单精度浮点数。
* nextPts ：输出二维点的矢量（具有单精度浮点坐标），包含第二图像中输入特征的计算新位置;当传
* OPTFLOW_USE_INITIAL_FLOW标志时，向量必须与输入中的大小相同。
* status ：输出状态向量（无符号字符）;如果找到相应特征的流，则向量的每个元素设置为1，否则设置为0。
* err ：输出错误的矢量; 向量的每个元素都设置为相应特征的错误，错误度量的类型可以在flags参数中设置; 如果未找到流，则未定义错误（使用status参数查找此类情况）。
* winSize ：每个金字塔等级的搜索窗口的winSize大小。
* maxLevel ：基于0的最大金字塔等级数;如果设置为0，则不使用金字塔（单级），如果设置为1，则使用两个级别，依此类推;如果将金字塔传递给输入，那么算法将使用与金字塔一样多的级别，但不超过maxLevel。
* criteria ：参数，指定迭代搜索算法的终止条件（在指定的最大迭代次数criteria.maxCount之后或当搜索窗移动小于criteria.epsilon时）。
* flags ：操作标志：OPTFLOW_USE_INITIAL_FLOW使用初始估计，存储在nextPts中;如果未设置标志，则将prevPts复制到nextPts并将其视为初始估计。OPTFLOW_LK_GET_MIN_EIGENVALS使用最小特征值作为误差测量（参见minEigThreshold描述）;如果没有设置标志，则将原稿周围的色块和移动点之间的L1距离除以窗口中的像素数，用作误差测量。
* minEigThreshold ：算法计算光流方程的2x2正常矩阵的最小特征值，除以窗口中的像素数;如果此值小于minEigThreshold，则过滤掉相应的功能并且不处理其流程，因此它允许删除坏点并获得性能提升。
  该函数实现了金字塔中Lucas-Kanade光流的稀疏迭代版本。




# - 其他

## kslam/config.h

```cpp

namespace kslam {

/**
 * 配置类，使用SetParameterFile确定配置文件
 * 然后用Get得到对应值
 * 单例模式
 */
    class Config {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {}  // private constructor makes a singleton
    public:
        ~Config();  // close the file when deconstructing

        // set a new config file
        static bool SetParameterFile(const std::string &filename);

        // access the parameter values
        template <typename T>
        static T Get(const std::string &key) {
            return T(Config::config_->file_[key]);
        }
    };
}  // namespace myslam

```

```cpp
#include "kslam/config.h"

namespace kslam {
    bool Config::SetParameterFile(const std::string &filename) {
        if (config_ == nullptr)
            config_ = std::shared_ptr<Config>(new Config);
        config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (config_->file_.isOpened() == false) {
            LOG(ERROR) << "parameter file " << filename << " does not exist.";
            config_->file_.release();
            return false;
        }
        return true;
    }
    Config::~Config() {
        if (file_.isOpened())
            file_.release();
    }
    std::shared_ptr<Config> Config::config_ = nullptr;
}

```

