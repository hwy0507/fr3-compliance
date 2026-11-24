// Camera-only recorder. No robot/network API. Native V4L2 capture timestamps.
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static volatile std::sig_atomic_t stopping=0;
static void stop(int){stopping=1;}
static int64_t mono(){timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t))throw std::runtime_error("clock_gettime");return int64_t(t.tv_sec)*1000000000+t.tv_nsec;}
static void call(int fd,unsigned long op,void* arg){int r;do{r=ioctl(fd,op,arg);}while(r<0&&errno==EINTR);if(r<0)throw std::runtime_error(std::string("V4L2: ")+strerror(errno));}
struct Device{
 int fd=-1;bool streaming=false;std::vector<std::pair<void*,size_t>> maps;
 ~Device(){if(streaming){int type=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(fd,VIDIOC_STREAMOFF,&type);}for(auto m:maps)munmap(m.first,m.second);if(fd>=0)close(fd);}
};
int main(int argc,char** argv){try{
 if(argc!=4){std::cerr<<"Camera only: capture_v4l2 DEVICE NEW_OUTPUT_DIR SECONDS\n";return 2;}
 const double seconds=std::stod(argv[3]);if(!(seconds>=1&&seconds<=120))throw std::runtime_error("Duration must be 1..120 seconds");
 const std::filesystem::path dir(argv[2]);if(!std::filesystem::create_directory(dir))throw std::runtime_error("Output directory must be new");
 if(std::filesystem::space(dir).available<uint64_t(seconds*640*480*2*30+128000000))throw std::runtime_error("Not enough disk space");
 std::string boot;{std::ifstream f("/proc/sys/kernel/random/boot_id");f>>boot;}if(boot.empty())throw std::runtime_error("Missing boot ID");
 Device dev;dev.fd=open(argv[1],O_RDWR|O_NONBLOCK|O_CLOEXEC);if(dev.fd<0)throw std::runtime_error(std::string("Cannot open camera: ")+strerror(errno));
 v4l2_capability cap{};call(dev.fd,VIDIOC_QUERYCAP,&cap);
 v4l2_format fmt{};fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;fmt.fmt.pix.width=640;fmt.fmt.pix.height=480;fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_YUYV;fmt.fmt.pix.field=V4L2_FIELD_ANY;
 call(dev.fd,VIDIOC_S_FMT,&fmt);
 if(fmt.fmt.pix.width!=640||fmt.fmt.pix.height!=480||fmt.fmt.pix.pixelformat!=V4L2_PIX_FMT_YUYV||fmt.fmt.pix.bytesperline<1280)throw std::runtime_error("640x480 YUYV required");
 v4l2_streamparm parm{};parm.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;parm.parm.capture.timeperframe.numerator=1;parm.parm.capture.timeperframe.denominator=30;call(dev.fd,VIDIOC_S_PARM,&parm);
 v4l2_requestbuffers req{};req.count=4;req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;req.memory=V4L2_MEMORY_MMAP;call(dev.fd,VIDIOC_REQBUFS,&req);if(req.count<2)throw std::runtime_error("Insufficient camera buffers");
 for(unsigned i=0;i<req.count;++i){v4l2_buffer b{};b.type=req.type;b.memory=req.memory;b.index=i;call(dev.fd,VIDIOC_QUERYBUF,&b);void* m=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,dev.fd,b.m.offset);if(m==MAP_FAILED)throw std::runtime_error("mmap failed");dev.maps.emplace_back(m,b.length);call(dev.fd,VIDIOC_QBUF,&b);}
 std::ofstream raw(dir/"camera.yuyv",std::ios::binary),csv(dir/"camera_frames.csv");
 if(!raw||!csv)throw std::runtime_error("Cannot create camera logs");
 csv<<"frame,sequence,capture_monotonic_ns,dequeue_monotonic_ns,flags,timestamp_source,raw_offset,raw_bytes\n";
 {std::ofstream f(dir/"clock.json");f<<"{\"schema\":\"FR3_CAMERA_MONOTONIC_V1\",\"clock\":\"CLOCK_MONOTONIC\",\"boot_id\":\""<<boot<<"\",\"width\":640,\"height\":480,\"pixel_format\":\"YUYV\",\"requested_fps\":30,\"robot_connected\":false}\n";}
 signal(SIGTERM,stop);signal(SIGINT,stop);
 int type=V4L2_BUF_TYPE_VIDEO_CAPTURE;call(dev.fd,VIDIOC_STREAMON,&type);dev.streaming=true;
 const int64_t begin=mono();int64_t previous=-1;unsigned frames=0;unsigned error_buffers=0;
 while(!stopping&&mono()-begin<int64_t(seconds*1e9)){
  pollfd p{dev.fd,POLLIN,0};int ready=poll(&p,1,1000);if(ready<0&&errno==EINTR)continue;if(ready<=0)throw std::runtime_error("Camera frame timeout");
  v4l2_buffer b{};b.type=type;b.memory=V4L2_MEMORY_MMAP;call(dev.fd,VIDIOC_DQBUF,&b);const int64_t received=mono();
  if((b.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)!=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)throw std::runtime_error("Camera does not report CLOCK_MONOTONIC timestamps; cannot claim synchronization");
  const int64_t stamp=int64_t(b.timestamp.tv_sec)*1000000000+int64_t(b.timestamp.tv_usec)*1000;
  if(stamp<=previous||stamp>received+10000000||received-stamp>1000000000)throw std::runtime_error("Invalid, stale or nonmonotonic camera timestamps");
  previous=stamp;
  if(b.index>=dev.maps.size()||b.bytesused>dev.maps[b.index].second)throw std::runtime_error("Invalid V4L2 buffer");
  if(b.flags&V4L2_BUF_FLAG_ERROR){++error_buffers;call(dev.fd,VIDIOC_QBUF,&b);continue;}
  const size_t stride=fmt.fmt.pix.bytesperline;if(b.bytesused<stride*479+1280)throw std::runtime_error("Truncated camera frame");
  const char* data=static_cast<const char*>(dev.maps[b.index].first);
  for(int y=0;y<480;++y)raw.write(data+y*stride,1280);
  const bool soe=(b.flags&V4L2_BUF_FLAG_TSTAMP_SRC_MASK)==V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
  csv<<frames<<','<<b.sequence<<','<<stamp<<','<<received<<','<<b.flags<<','<<(soe?"SOE":"EOF")<<','<<uint64_t(frames)*640*480*2<<','<<640*480*2<<'\n';
  if(!raw||!csv)throw std::runtime_error("Camera log write failed");
  ++frames;call(dev.fd,VIDIOC_QBUF,&b);
  if(frames==1){csv.flush();std::ofstream f(dir/"READY");f<<stamp<<'\n';std::cout<<"CAMERA_READY CLOCK_MONOTONIC "<<(soe?"SOE":"EOF")<<std::endl;}
 }
 raw.flush();csv.flush();if(!raw||!csv||!frames)throw std::runtime_error("Empty/failed camera recording");
 std::ofstream f(dir/"complete.json");f<<"{\"frames\":"<<frames<<",\"error_buffers\":"<<error_buffers<<",\"stopped_by_signal\":"<<(stopping?"true":"false")<<"}\n";
 std::cout<<"Camera complete: "<<frames<<" frames, "<<error_buffers<<" error buffers\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<"CAMERA_FAILED: "<<e.what()<<'\n';return 1;}}
