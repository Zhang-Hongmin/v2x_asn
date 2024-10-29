doc：是v2x相关标准文档《YDT 3709-2020 基于LTE的车联网无线通信技术 消息层技术要求》...
examples：是消息集编解码示例
v2x_asn：是根据.asn文件生成的源码，后续更新.asn文件即可更新源码
v2x_api：是基于v2x_asn源码封装的v2x asn uper编解码函数接口
V2X-2020.asn：是从《YDT 3709-2020 基于LTE的车联网无线通信技术...》中复制出的asn代码内容

一、编译测试步骤：
1、编译.so，执行命令：
make clean
make 

2、编译测试demo，执行命令：
make test
export LD_LIBRARY_PATH=$(pwd)
./test1

二、V2X消息集开发流程补充说明--------------------------------------------：
参照：https://blog.csdn.net/zhoubiaodi/article/details/132424966
1、从《YDT 3709-2020 基于LTE的车联网无线通信技术 消息层技术要求》文档复制出asn代码内容，创建1个.asn文件
2、安装asn.1编译工具
这里选用免费开源的asn1c编译工具(这个工具是ASN.1的c/c++实例)
安装方法1，在线下载安装，直接输入命令即可：apt-get install asn1c
安装方法2，源码编译安装：
asn1c官网源码下载地址：http://lionet.info/asn1c/download.html
或者直接输入命令下载：git clone git://github.com/vlm/asn1c.git
编译安装：（参考该目录文件INSTALL.md）
test -f configure || autoreconf -iv
./configure
make
make install

3、编译.asn文件
使用命令编译.asn文件，生成对应代码文件（*.c 和.h文件，有对应的数据结构体）：
创建文件夹：
mkdir v2x_asn
cp V2X-2020.asn v2x_asn/
cd  v2x_asn/

编译:（要加参数-gen-PER，表示支持PER编码，否则后续调用API实现ASN1的UPER编解码不通过）
asn1c -gen-PER V2X-2020.asn

里面有个sample示例，避免影响编译库，记得把它删除或者改名，输入：
mv converter-sample.c converter-sample.c_bak

4、调用API实现ASN1的UPER编解码（引用生成的代码文件，调用API接口和结构体）（参考examples/test1.c）
填充结构体数据
编码：使用uper_encode_to_buffer函数对结构体进行编码，得到字节流数据（1个buf，后续可以将buf数据通过各种通信方式发送给别人）
解码：使用uper_decode函数对接收到的buf数据进行解码，还原结构体，得到结构体里面的参数内容
