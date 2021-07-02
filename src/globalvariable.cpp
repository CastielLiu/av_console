#include "../include/globalvariables.hpp"
/*全局变量*/

/*@brief RosNodes: 存放ros nodes信息(launch)
 *       RosNodesArray: 存放多个launch文件信息
 */
av_console::RosNodesArray g_rosNodesArray;

/*@brief Log 系统日志
 */
av_console::Logg g_logg;  //g_log这个变量名竟然会和QApplication app(argc,argv)冲突！
