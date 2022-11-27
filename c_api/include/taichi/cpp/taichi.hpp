// C++ wrapper of Taichi C-API
#pragma once
#include <cstring>
#include <list>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include "taichi/taichi.h"

namespace ti {

// Token type for half-precision floats.
struct half {      //half是一个半精度的浮点数
  uint16_t _;
};

namespace detail {

// Template type to data type enum.
template <typename T>
struct templ2dtype {};          //一个模板以及下面的一些特化版本
template <>
struct templ2dtype<int8_t> {
  static const TiDataType value = TI_DATA_TYPE_I8;    //int8 int16 int32还有浮点等等的一些东西
};
template <>
struct templ2dtype<int16_t> {
  static const TiDataType value = TI_DATA_TYPE_I16;
};
template <>
struct templ2dtype<int32_t> {
  static const TiDataType value = TI_DATA_TYPE_I32;
};
template <>
struct templ2dtype<uint8_t> {
  static const TiDataType value = TI_DATA_TYPE_U8;
};
template <>
struct templ2dtype<uint16_t> {
  static const TiDataType value = TI_DATA_TYPE_U16;
};
template <>
struct templ2dtype<uint32_t> {
  static const TiDataType value = TI_DATA_TYPE_U32;
};
template <>
struct templ2dtype<half> {
  static const TiDataType value = TI_DATA_TYPE_F16;
};
template <>
struct templ2dtype<float> {
  static const TiDataType value = TI_DATA_TYPE_F32;
};
template <>
struct templ2dtype<double> {
  static const TiDataType value = TI_DATA_TYPE_F64;
};

template <typename THandle>                       //又是一个模板函数,将一个handle move为右值,然后返回
THandle move_handle(THandle &handle) {
  THandle out = std::move(handle);
  handle = TI_NULL_HANDLE;
  return out;
}

}  // namespace detail

class Memory {                                    //一个很大的内存类
  TiRuntime runtime_{TI_NULL_HANDLE};             //有运行时,包含device和context
  TiMemory memory_{TI_NULL_HANDLE};               //底层包含Timemory
  size_t size_{0};                                //内存大小
  bool should_destroy_{false};                    //是否应该销毁

 public:
  constexpr bool is_valid() const {               //该内存是否有效,判断依据是runtime是否有效
    return runtime_ != nullptr;
  }
  inline void destroy() {                         //销毁内存的方法,是在底层调用一个内存释放函数
                                                  //然后把Timemory置空
    if (should_destroy_) {
      ti_free_memory(runtime_, memory_);
      memory_ = TI_NULL_HANDLE;
      should_destroy_ = false;
    }
  }

  Memory() {                                      //默认构造,说明在时机的生产生活中,一般是不咋实现默认构造的,但是还是要写一个,防止编译器自己生成构造函数
  }
  Memory(const Memory &) = delete;                //禁用拷贝构造
  Memory(Memory &&b)                              //移动构造可以有,使用上面提到的移动handle函数,当一个右值memory传进来之后,把他所有的重要成员都move之后再进行赋值,用列表初始化的方法
      : runtime_(detail::move_handle(b.runtime_)),
        memory_(detail::move_handle(b.memory_)),
        size_(std::exchange(b.size_, 0)),
        should_destroy_(std::exchange(b.should_destroy_, false)) {
  }
  Memory(TiRuntime runtime, TiMemory memory, size_t size, bool should_destroy)   //有参构造,传入runtime,Timemory,大小以及是否应该销毁4个参数,就可以构造一个Memory
      : runtime_(runtime),
        memory_(memory),
        size_(size),
        should_destroy_(should_destroy) {
  }
  ~Memory() {                   //析构函数,调用自己创建的一个销毁函数
    destroy();
  }

  Memory &operator=(const Memory &) = delete;                         //赋值运算符,拷贝赋值依然不能有,移动赋值可以有
  Memory &operator=(Memory &&b) {                                     
    destroy();                                                        //对一个Memory类执行移动赋值操作之前,先把自己本身销毁掉,再把右值相关的东西赋值进来,最后返回本体
    runtime_ = detail::move_handle(b.runtime_);
    memory_ = detail::move_handle(b.memory_);
    size_ = std::exchange(b.size_, 0);
    should_destroy_ = std::exchange(b.should_destroy_, false);
    return *this;
  }

  void *map() const {                                                 //映射?映射什么内存,不太清楚
    return ti_map_memory(runtime_, memory_);
  }
  void unmap() const {
    ti_unmap_memory(runtime_, memory_);
  }

  inline void read(void *dst, size_t size) const {                    //输入一个目的地指针,以及大小,直接先一波内存映射拿到内存类最底层的真正内存的指针,然后执行内存拷贝,把Memory内存中真正
                                                                      //的东西存进我们的目的内存当中          
    void *src = map();
    if (src != nullptr) {
      std::memcpy(dst, src, size);
    }
    unmap();
  }
  inline void write(const void *src, size_t size) const {             //写函数,就是类似的道理,同样进行内存映射,但是是给了一个源地址,将源地址内的东西拷贝到Memory当中。
    void *dst = map();
    if (dst != nullptr) {
      std::memcpy(dst, src, size);
    }
    unmap();
  }

  constexpr size_t size() const {                                       //类设计这一定会写一些这样的接口,从而获取到私有的成员属性,这也让我学到了一个编程规范,但是不一定到哪家公司都是这样的。
                                                                        //就是私有属性命名的时候带一个下划线,然后命名获取接口的时候可以命名成符合我们直觉的名字
    return size_;
  }
  constexpr TiMemory memory() const {
    return memory_;
  }
  constexpr operator TiMemory() const {
    return memory_;
  }
};

template <typename T>
class NdArray {                                                               //NdArray里面主要的成员就是一个内存和一个TiNdArray
  Memory memory_{};
  TiNdArray ndarray_{};

 public:
  constexpr bool is_valid() const {                                           //判断一个NdArray是否有效的标准就是看内存是否有效,这就开始一层套一层了,好恶心
    return memory_.is_valid();
  }
  inline void destroy() {                                                     //内敛一个销毁函数,其实就是销毁内存
    memory_.destroy();
  }

  NdArray() {                                                                 //同样的占位用的默认构造
  } 
  NdArray(const NdArray<T> &) = delete;                                       //同样的不允许拷贝
  NdArray(NdArray<T> &&b)
      : memory_(std::move(b.memory_)), ndarray_(std::exchange(b.ndarray_, {})) {          //移动构造,可以把内存移动过来,**std::exchange()其实也是一个移动语义,相当于用一个右值
                                                                                          //把原始数组的内容拿过来,再用一个空的数组给填补上
  }
  NdArray(Memory &&memory, const TiNdArray &ndarray)                                      //或者有参构造
      : memory_(std::move(memory)), ndarray_(ndarray) {
    if (ndarray.memory != memory_) {
      ti_set_last_error(TI_ERROR_INVALID_ARGUMENT, "ndarray.memory != memory");
    }
  }
  ~NdArray() {
    destroy();
  }

  NdArray<T> &operator=(const NdArray<T> &) = delete;                                     //凡是和拷贝有关都默认没有
  NdArray<T> &operator=(NdArray<T> &&b) {                                                 //移动语义可以有
    destroy();
    memory_ = std::move(b.memory_);
    ndarray_ = std::exchange(b.ndarray_, {});
    return *this;
  }

  inline void *map() const {                    
    return memory_.map();
  }
  inline void unmap() const {
    return memory_.unmap();
  }

  inline void read(T *dst, size_t count) const {                                          //同样的读内存
    memory_.read(dst, count * sizeof(T));       
  }
  inline void read(std::vector<T> &dst) const {                                           //同样的写内存
    read(dst.data(), dst.size());
  }
  template <typename U>
  inline void read(std::vector<U> &dst) const {                                           //模板函数,这里的T是整个NdArray的模板参数
    static_assert(sizeof(U) % sizeof(T) == 0,
                  "sizeof(U) must be a multiple of sizeof(T)");
    read((T *)dst.data(), dst.size() * (sizeof(U) / sizeof(T)));
  }
  inline void write(const T *src, size_t count) const {                                   //底层调用写入函数
    memory_.write(src, count * sizeof(T));
  }
  inline void write(const std::vector<T> &src) const {
    write(src.data(), src.size());
  }
  template <typename U>
  inline void write(const std::vector<U> &src) const {
    static_assert(sizeof(U) % sizeof(T) == 0,
                  "sizeof(U) must be a multiple of sizeof(T)");
    write((const T *)src.data(), src.size() * (sizeof(U) / sizeof(T)));
  }

  constexpr TiDataType elem_type() const {                                                //返回Ndarray的元素类型
    return ndarray_.elem_type;
  }
  constexpr const TiNdShape &shape() const {                                              //NdArray的尺寸
    return ndarray_.shape;
  }
  constexpr const TiNdShape &elem_shape() const {                                         //元素的尺寸
    return ndarray_.elem_shape;
  }
  constexpr const Memory &memory() const {                                                //返回array的内存
    return memory_;
  }
  constexpr const TiNdArray &ndarray() const {                                        
    return ndarray_;
  }
  constexpr operator TiNdArray() const {
    return ndarray_;
  }
};

class Image {
  TiRuntime runtime_{TI_NULL_HANDLE};                                                         //默认为空
  TiImage image_{TI_NULL_HANDLE};
  bool should_destroy_{false};

 public:
  constexpr bool is_valid() const {                                                           //_image来判断Image是否有效。
    return image_ != nullptr;
  }
  inline void destroy() {
    if (should_destroy_) {                                                                    //释放图像,以及成员函数置空
      ti_free_image(runtime_, image_);
      image_ = TI_NULL_HANDLE;
      should_destroy_ = false;
    }
  }

  Image() {
  }
  Image(const Image &b) = delete;                                                             //拷贝构造没有
  Image(Image &&b)                                                                            //移动构造
      : runtime_(detail::move_handle(b.runtime_)),
        image_(detail::move_handle(b.image_)),
        should_destroy_(std::exchange(b.should_destroy_, false)) {
  }
  Image(TiRuntime runtime, TiImage image, bool should_destroy)                                 //有参构造
      : runtime_(runtime), image_(image), should_destroy_(should_destroy) {
  }
  ~Image() {
    destroy();
  }

  Image &operator=(const Image &) = delete;                                                     //移动赋值运算符的固定使用套路,先析构
                                                                                                //移动右值,给当前对象赋值
  Image &operator=(Image &&b) {
    destroy();
    runtime_ = detail::move_handle(b.runtime_);
    image_ = detail::move_handle(b.image_);
    should_destroy_ = std::exchange(b.should_destroy_, false);
    return *this;
  }

  TiImageSlice slice(TiImageOffset offset,                                                       //构建一个切片,使用偏移量,尺寸,mipmap尺寸
                     TiImageExtent extent,
                     uint32_t mip_level) const {
    TiImageSlice slice{};
    slice.image = image_;
    slice.extent = extent;
    slice.offset = offset;
    slice.mip_level = mip_level;
    return slice;
  }

  constexpr TiImage image() const {
    return image_;
  }
  constexpr operator TiImage() const {
    return image_;
  }
};

class Texture {
  Image image_{};
  TiTexture texture_{};

 public:
  constexpr bool is_valid() const {                                               //纹理自带一个图像
    return image_.is_valid();
  }
  inline void destroy() {
    image_.destroy();
  }

  Texture() {                                                                     //默认构造灭有
  }
  Texture(const Texture &b) = delete;
  Texture(Texture &&b)
      : image_(std::move(b.image_)), texture_(std::move(b.texture_)) {
  }
  Texture(Image &&image, const TiTexture &texture)
      : image_(std::move(image)), texture_(texture) {
    if (texture.image != image_) {
      ti_set_last_error(TI_ERROR_INVALID_ARGUMENT, "texture.image != image");                 //初始化纹理的时候
    }
  }
  ~Texture() {
    destroy();
  }

  Texture &operator=(const Texture &) = delete;
  Texture &operator=(Texture &&b) {
    destroy();
    image_ = std::move(b.image_);
    texture_ = std::move(b.texture_);
    return *this;
  }

  constexpr const Image &image() const {
    return image_;
  }
  constexpr TiTexture texture() const {
    return texture_;
  }
  constexpr operator TiTexture() const {
    return texture_;
  }
};

class ArgumentEntry {                           //参数条目          
  friend class ComputeGraph;                    //计算图
  TiArgument *arg_;                             //参数指针,应该是在某一块内存当中的一系列参数

 public:
  ArgumentEntry() = delete;
  ArgumentEntry(const ArgumentEntry &) = delete;
  ArgumentEntry(ArgumentEntry &&b) : arg_(b.arg_) {
  }
  ArgumentEntry(TiArgument *arg) : arg_(arg) {
  }

  inline ArgumentEntry &operator=(const TiArgument &b) {
    *arg_ = b;
    return *this;
  }
  inline ArgumentEntry &operator=(int32_t i32) {
    arg_->type = TI_ARGUMENT_TYPE_I32;
    arg_->value.i32 = i32;
    return *this;
  }
  inline ArgumentEntry &operator=(float f32) {
    arg_->type = TI_ARGUMENT_TYPE_F32;
    arg_->value.f32 = f32;
    return *this;
  }
  inline ArgumentEntry &operator=(const TiNdArray &ndarray) {
    arg_->type = TI_ARGUMENT_TYPE_NDARRAY;
    arg_->value.ndarray = ndarray;
    return *this;
  }
  inline ArgumentEntry &operator=(const TiTexture &texture) {
    arg_->type = TI_ARGUMENT_TYPE_TEXTURE;
    arg_->value.texture = texture;
    return *this;
  }
};

class ComputeGraph {                                      //计算图里面包含runtime
  TiRuntime runtime_{TI_NULL_HANDLE};                    
  TiComputeGraph compute_graph_{TI_NULL_HANDLE};              
  std::list<std::string> arg_names_{};  // For stable addresses.                //参数名
  std::vector<TiNamedArgument> args_{};                                          //参数数组

 public:
  constexpr bool is_valid() const {
    return compute_graph_ != nullptr;
  }

  ComputeGraph() {
  }
  ComputeGraph(const ComputeGraph &) = delete;
  ComputeGraph(ComputeGraph &&b)
      : runtime_(detail::move_handle(b.runtime_)),
        compute_graph_(detail::move_handle(b.compute_graph_)),
        arg_names_(std::move(b.arg_names_)),
        args_(std::move(b.args_)) {
  }
  ComputeGraph(TiRuntime runtime, TiComputeGraph compute_graph)
      : runtime_(runtime), compute_graph_(compute_graph) {
  }
  ~ComputeGraph() {
  }

  ComputeGraph &operator=(const ComputeGraph &) = delete;
  ComputeGraph &operator=(ComputeGraph &&b) {
    runtime_ = detail::move_handle(b.runtime_);
    compute_graph_ = detail::move_handle(b.compute_graph_);
    arg_names_ = std::move(b.arg_names_);
    args_ = std::move(b.args_);
    return *this;
  }

  inline ArgumentEntry at(const char *name) {                   //我们使用这样一个内联函数,就可以实现根据参数名返回参数本身
    size_t i = 0;
    auto it = arg_names_.begin();
    for (; it != arg_names_.end(); ++it) {
      if (*it == name) {
        break;
      }
      ++i;
    }                                                           //找到了对应参数的迭代器

    TiArgument *out;                                            //创建一个参数
    if (it != arg_names_.end()) {                               //如果迭代器不是.end()
      out = &args_.at(i).argument;                              //设置这个参数指针指向那个参数的地址
    } else {                                                    //
      arg_names_.emplace_back(name);                            //如果啥也没找见,我们就把这个名字加入到里头,但我觉得这不合适吧,如果我瞎写一个名字那岂不是也会加入进来了
      args_.emplace_back();
      args_.back().name = arg_names_.back().c_str();
      out = &args_.back().argument;
    }

    return ArgumentEntry(out);
  };
  inline ArgumentEntry at(const std::string &name) {            //输入string也行,转换为c_str
    return at(name.c_str());
  }
  inline ArgumentEntry operator[](const char *name) {           //方括号也行
    return at(name);
  }
  inline ArgumentEntry operator[](const std::string &name) {
    return at(name);
  }

  void launch(uint32_t argument_count, const TiNamedArgument *arguments) {      //启动计算图,给出你的参数数量和参数指针,进行启动计算图
    ti_launch_compute_graph(runtime_, compute_graph_, argument_count,
                            arguments);
  }
  void launch() {                                                               //没给参数直接传入所有参数进行调用
    launch(args_.size(), args_.data());
  }
  void launch(const std::vector<TiNamedArgument> &arguments) {                          //也可以只传一个参数数组
    launch(arguments.size(), arguments.data());
  }

  constexpr TiComputeGraph compute_graph() const {
    return compute_graph_;
  }
  constexpr operator TiComputeGraph() const {                                           //好像是临时对象的感觉
    return compute_graph_;
  }
};

class Kernel {                                                                          //核的概念
  TiRuntime runtime_{TI_NULL_HANDLE};                                                  //一个运行时,一个Ti核,一个参数表  
  TiKernel kernel_{TI_NULL_HANDLE};
  std::vector<TiArgument> args_{};

 public:
  constexpr bool is_valid() const {                                                         //_Kernel有才算真的有
    return kernel_ != nullptr;
  }

  Kernel() {
  }
  Kernel(const Kernel &) = delete;
  Kernel(Kernel &&b)
      : runtime_(detail::move_handle(b.runtime_)),
        kernel_(detail::move_handle(b.kernel_)),
        args_(std::move(b.args_)) {
  }
  Kernel(TiRuntime runtime, TiKernel kernel)
      : runtime_(runtime), kernel_(kernel) {
  }

  Kernel &operator=(const Kernel &) = delete;
  Kernel &operator=(Kernel &&b) {
    runtime_ = detail::move_handle(b.runtime_);
    kernel_ = detail::move_handle(b.kernel_);
    args_ = std::move(b.args_);
    return *this;
  }

  ArgumentEntry at(uint32_t i) {
    if (i < args_.size()) {
      return ArgumentEntry(&args_.at(i));
    } else {
      args_.resize(i + 1);
      return ArgumentEntry(&args_.at(i));
    }
  }
  ArgumentEntry operator[](uint32_t i) {
    return at(i);
  }

  template <typename T>
  void push_arg(const std::vector<T> &v) {
    int idx = args_.size();
    // Temporary workaround for setting vec/matrix arguments in a flattened way.
    args_.resize(args_.size() + v.size());
    for (int j = 0; j < v.size(); ++j) {
      at(idx + j) = v[j];
    }
  }

  template <typename T>
  void push_arg(const T &arg) {
    int idx = args_.size();
    args_.resize(idx + 1);
    at(idx) = arg;
  }

  void clear_args() {
    args_.clear();
  }

  void launch(uint32_t argument_count, const TiArgument *arguments) {                   //kernel的启动方式和计算图类似,可能计算图启动的底层就是核启动
    ti_launch_kernel(runtime_, kernel_, argument_count, arguments);
  }
  void launch() {
    launch(args_.size(), args_.data());
  }
  void launch(const std::vector<TiArgument> &arguments) {
    launch(arguments.size(), arguments.data());
  }

  constexpr TiKernel kernel() const {
    return kernel_;
  }
  constexpr operator TiKernel() const {
    return kernel_;
  }
};

class AotModule {                                                                 //AOT模块,主要干的事情就是里面有kernel和计算图
  TiRuntime runtime_{TI_NULL_HANDLE};
  TiAotModule aot_module_{TI_NULL_HANDLE};
  bool should_destroy_{false};

 public:
  constexpr bool is_valid() const {
    return aot_module_ != nullptr;
  }
  inline void destroy() {
    if (should_destroy_) {
      ti_destroy_aot_module(aot_module_);
      aot_module_ = TI_NULL_HANDLE;
      should_destroy_ = false;
    }
  }

  AotModule() {
  }
  AotModule(const AotModule &) = delete;
  AotModule(AotModule &&b)
      : runtime_(detail::move_handle(b.runtime_)),
        aot_module_(detail::move_handle(b.aot_module_)),
        should_destroy_(std::exchange(b.should_destroy_, false)) {
  }
  AotModule(TiRuntime runtime, TiAotModule aot_module, bool should_destroy)
      : runtime_(runtime),
        aot_module_(aot_module),
        should_destroy_(should_destroy) {
  }
  ~AotModule() {
    destroy();
  }

  AotModule &operator=(const AotModule &) = delete;
  AotModule &operator=(AotModule &&b) {
    runtime_ = detail::move_handle(b.runtime_);
    aot_module_ = detail::move_handle(b.aot_module_);
    should_destroy_ = std::exchange(b.should_destroy_, false);
    return *this;
  }

  Kernel get_kernel(const char *name) {
    TiKernel kernel_ = ti_get_aot_module_kernel(aot_module_, name);                           //可以直接凭借名字从AOT模块里面把kernel拿出来
    return Kernel(runtime_, kernel_);
  }
  ComputeGraph get_compute_graph(const char *name) {                                        
    TiComputeGraph compute_graph_ =
        ti_get_aot_module_compute_graph(aot_module_, name);
    return ComputeGraph(runtime_, compute_graph_);
  }

  constexpr TiAotModule aot_module() const {
    return aot_module_;
  }
  constexpr operator TiAotModule() const {
    return aot_module_;
  }
};

class Event {                                                                                       //事件
  TiRuntime runtime_{TI_NULL_HANDLE};
  TiEvent event_{TI_NULL_HANDLE};
  bool should_destroy_{false};

 public:
  constexpr bool is_valid() const {
    return event_ != nullptr;
  }
  inline void destroy() {
    if (should_destroy_) {
      ti_destroy_event(event_);
      event_ = TI_NULL_HANDLE;
      should_destroy_ = false;
    }
  }

  Event() {
  }
  Event(const Event &) = delete;
  Event(Event &&b) : event_(b.event_), should_destroy_(b.should_destroy_) {
  }
  Event(TiRuntime runtime, TiEvent event, bool should_destroy)
      : runtime_(runtime), event_(event), should_destroy_(should_destroy) {
  }
  ~Event() {
    destroy();
  }

  Event &operator=(const Event &) = delete;
  Event &operator=(Event &&b) {
    event_ = detail::move_handle(b.event_);
    should_destroy_ = std::exchange(b.should_destroy_, false);
    return *this;
  }

  void reset(TiEvent event_) {
    ti_reset_event(runtime_, event_);
  }
  void signal(TiEvent event_) {
    ti_signal_event(runtime_, event_);                                  //对runtime指令signal和wait两个过程
  }
  void wait(TiEvent event_) {
    ti_wait_event(runtime_, event_);
  }

  constexpr TiEvent event() const {
    return event_;
  }
  constexpr operator TiEvent() const {
    return event_;
  }
};

class Runtime {
  TiArch arch_{TI_ARCH_MAX_ENUM};                                             //框架,这里应该是可以选择vulkan版,mental版,CUDA版等等的backend
  TiRuntime runtime_{TI_NULL_HANDLE};                                         //以及一个TiRuntime
  bool should_destroy_{false};

 public:
  constexpr bool is_valid() const {
    return runtime_ != nullptr;
  }
  inline void destroy() {
    if (should_destroy_) {
      ti_destroy_runtime(runtime_);
      runtime_ = TI_NULL_HANDLE;
      should_destroy_ = false;
    }
  }

  Runtime() {
  }
  Runtime(const Runtime &) = delete;
  Runtime(Runtime &&b)
      : arch_(std::exchange(b.arch_, TI_ARCH_MAX_ENUM)),
        runtime_(detail::move_handle(b.runtime_)),
        should_destroy_(std::exchange(b.should_destroy_, false)) {
  }
  Runtime(TiArch arch)
      : arch_(arch), runtime_(ti_create_runtime(arch)), should_destroy_(true) {
  }
  Runtime(TiArch arch, TiRuntime runtime, bool should_destroy)
      : arch_(arch), runtime_(runtime), should_destroy_(should_destroy) {
  }
  ~Runtime() {
    destroy();
  }

  Runtime &operator=(const Runtime &) = delete;
  Runtime &operator=(Runtime &&b) {
    arch_ = std::exchange(b.arch_, TI_ARCH_MAX_ENUM);
    runtime_ = detail::move_handle(b.runtime_);
    should_destroy_ = std::exchange(b.should_destroy_, false);
    return *this;
  }

  std::map<TiCapability, uint32_t> get_capabilities() const {
    uint32_t n = 0;
    ti_get_runtime_capabilities(runtime_, &n, nullptr);                               //根据backend,有一堆能力,这里有点类似vulkan的设计模式
    std::vector<TiCapabilityLevelInfo> devcaps(n);
    ti_get_runtime_capabilities(runtime_, &n, devcaps.data());

    std::map<TiCapability, uint32_t> out{};                                       
    for (auto devcap : devcaps) {
      out[devcap.capability] = devcap.level;
    }
    return out;
  }

  Memory allocate_memory(const TiMemoryAllocateInfo &allocate_info) {                     //分配内存,通过分配信息来分配
    TiMemory memory = ti_allocate_memory(runtime_, &allocate_info);
    return Memory(runtime_, memory, allocate_info.size, true);
  } 
  Memory allocate_memory(size_t size) {                                                    //或者直接用大小来分配
    TiMemoryAllocateInfo allocate_info{};
    allocate_info.size = size;
    allocate_info.usage = TI_MEMORY_USAGE_STORAGE_BIT;
    return allocate_memory(allocate_info);
  }
  template <typename T>
  NdArray<T> allocate_ndarray(const std::vector<uint32_t> &shape = {},                      //可以分配数组
                              const std::vector<uint32_t> &elem_shape = {},
                              bool host_access = false) {
    size_t size = sizeof(T);
    TiNdArray ndarray{};
    for (size_t i = 0; i < shape.size(); ++i) {
      uint32_t x = shape.at(i);
      size *= x;                                              //这里面还在统计整个NdArray的大小
      ndarray.shape.dims[i] = x;                              //用数组存储维度,因此理论上无限维,这里确定了每一维的尺寸
    }
    ndarray.shape.dim_count = shape.size();                   //这里知道了维数                
    for (size_t i = 0; i < elem_shape.size(); ++i) {          //又在确定里面元素的维数,和具体每一个维度的尺寸
      uint32_t x = elem_shape.at(i);
      size *= x;
      ndarray.elem_shape.dims[i] = x;
    }
    ndarray.elem_shape.dim_count = elem_shape.size();
    ndarray.elem_type = detail::templ2dtype<T>::value;            //初始化数据类型

    TiMemoryAllocateInfo allocate_info{};                         //开始为array分配内存
    allocate_info.size = size;
    allocate_info.host_read = host_access;
    allocate_info.host_write = host_access;
    allocate_info.usage = TI_MEMORY_USAGE_STORAGE_BIT;
    Memory memory = allocate_memory(allocate_info);
    ndarray.memory = memory;
    return NdArray<T>(std::move(memory), ndarray);
  }

  Image allocate_image(const TiImageAllocateInfo &allocate_info) {    //分配Image
    TiImage image = ti_allocate_image(runtime_, &allocate_info);
    return Image(runtime_, image, true);
  }
  Texture allocate_texture2d(uint32_t width,
                             uint32_t height,
                             TiFormat format,
                             TiSampler sampler) {                       //分配二维纹理,宽高格式,采样方式都要设置
    TiImageExtent extent{};                                             //这里一看就是任冬写的,太厉害了
    extent.width = width;
    extent.height = height;
    extent.depth = 1;
    extent.array_layer_count = 1;                                        

    TiImageAllocateInfo allocate_info{};
    allocate_info.dimension = TI_IMAGE_DIMENSION_2D;
    allocate_info.extent = extent;
    allocate_info.mip_level_count = 1;
    allocate_info.format = format;
    allocate_info.usage =
        TI_IMAGE_USAGE_STORAGE_BIT | TI_IMAGE_USAGE_SAMPLED_BIT;

    Image image = allocate_image(allocate_info);                      //根据info创建图像
    TiTexture texture{};                                              //创建纹理,有了这些东西
    texture.image = image;
    texture.dimension = TI_IMAGE_DIMENSION_2D;
    texture.extent = extent;
    texture.format = format;
    texture.sampler = sampler;
    return Texture(std::move(image), texture);                        //临时对象初始化之后,直接右值初始化一个返回
  }

  AotModule load_aot_module(const char *path) {                       //runtime里面可以加载AOT模块
    TiAotModule aot_module_ = ti_load_aot_module(runtime_, path);
    return AotModule(runtime_, aot_module_, true);
  }
  AotModule load_aot_module(const std::string &path) {                //怎么加载呢?直接由模块所处的路径找见直接加载。
    return load_aot_module(path.c_str());
  }

  void copy_memory_device_to_device(const TiMemorySlice &dst_memory,                          //设备之间相互拷贝内存
                                    const TiMemorySlice &src_memory) {
    ti_copy_memory_device_to_device(runtime_, &dst_memory, &src_memory);
  }
  void copy_image_device_to_device(const TiImageSlice &dst_texture,
                                   const TiImageSlice &src_texture) {
    ti_copy_image_device_to_device(runtime_, &dst_texture, &src_texture);
  }
  void transition_image(TiImage image, TiImageLayout layout) {
    ti_transition_image(runtime_, image, layout);
  }

  void submit() {                                                                           //提交
    ti_submit(runtime_);
  }
  void wait() {                                                                             //等待
    ti_wait(runtime_);
  }

  constexpr TiArch arch() const {
    return arch_;
  }
  constexpr TiRuntime runtime() const {
    return runtime_;
  }
  constexpr operator TiRuntime() const {
    return runtime_;
  }
};

}  // namespace ti
