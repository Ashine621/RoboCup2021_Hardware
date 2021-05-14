/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

// 抓取参数调节（单位：米）(Modified in wpb_home.yaml!!)
static float grab_y_offset = -0.04f;          //抓取前，对准物品，机器人的横向位移偏移量
static float grab_lift_offset = 0.0f;       //手臂抬起高度的补偿偏移量
static float grab_forward_offset = 0.065f;    //手臂抬起后，机器人向前抓取物品移动的位移偏移量
static float grab_gripper_value = 0.0f;    //抓取物品时，手爪闭合后的手指间距

static float vel_max = 0.5;                     //移动限速

#define STEP_WAIT           0
#define STEP_FIND_PLANE     1
#define STEP_PLANE_DIST     2
#define STEP_FIND_OBJ       3
#define STEP_OBJ_DIST       4
#define STEP_HAND_UP        5
#define STEP_FORWARD        6
#define STEP_GRAB           7
#define STEP_OBJ_UP         8
#define STEP_BACKWARD       9
#define STEP_DONE           10
static int nStep = STEP_WAIT;

static std::string pc_topic;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static ros::Publisher result_pub;

void VelCmd(float inVx , float inVy, float inTz);


static std_msgs::String result_msg;

static ros::Publisher odom_ctrl_pub;
static std_msgs::String ctrl_msg;
static geometry_msgs::Pose2D pose_diff;

static float fObjGrabX = 0;
static float fObjGrabY = 0;
static float fObjGrabZ = 0;
static float fMoveTargetX = 0;
static float fMoveTargetY = 0;

static int nTimeDelayCounter = 0;

static float fTargetGrabX = 0.9;        //抓取时目标物品的x坐标
static float fTargetGrabY = 0.0;        //抓取时目标物品的y坐标

void GrabActionCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 目标物品的坐标
    fObjGrabX = msg->position.x;
    fObjGrabY = msg->position.y;
    fObjGrabZ = msg->position.z;
    ROS_WARN("[OBJ_TO_GRAB] x = %.2f y= %.2f ,z= %.2f " ,fObjGrabX, fObjGrabY, fObjGrabZ);
    ctrl_msg.data = "pose_diff reset";
    odom_ctrl_pub.publish(ctrl_msg); // 发现物品就重置里程计差分值

    // ajudge the dist to obj
    fMoveTargetX = fObjGrabX - fTargetGrabX; //机器人期望向前移动的距离
    fMoveTargetY = fObjGrabY - fTargetGrabY + grab_y_offset; //机器人期望横向移动的距离
    ROS_WARN("[MOVE_TARGET] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
    nStep = STEP_OBJ_DIST;
}

void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr& msg) //将订阅到的/wpb_home/pose_diff话题中的值赋值给pose_diff(机器人相对起始坐标的位姿变化值)
{
    pose_diff.x = msg->x;
    pose_diff.y = msg->y;
    pose_diff.theta = msg->theta;
}

float VelFixed(float inVel,float inMax) //滤波器。传入速度值，最大绝对值为0.5，超过0.5的速度值转化成0.5
{
    float retVel = inVel;
    if(retVel > inMax)
        retVel = inMax;
    if(retVel < -inMax)
        retVel = -inMax;
    return retVel;
}

void VelCmd(float inVx , float inVy, float inTz) //发布速度值，使用VelFixed函数将传入的速度值进行最大值为0.5限速之后发布
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = VelFixed(inVx , vel_max);
    vel_cmd.linear.y = VelFixed(inVy , vel_max);
    vel_cmd.angular.z = VelFixed(inTz , vel_max);
    vel_pub.publish(vel_cmd);
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg) // 这个msg可以让抓取停下
{
    int nFindIndex = msg->data.find("grab stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[grab_stop] ");
        nStep = STEP_WAIT;
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
        vel_pub.publish(vel_cmd);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_grab_action");
    ROS_INFO("wpb_home_grab_action");

    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30); // 发布底盘速度
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30); // 发布机械臂运动 JointState里面是几个vector，存了机械臂Joints的名字、高度和移动速度
    result_pub = nh.advertise<std_msgs::String>("/wpb_home/grab_result", 30); // 向外发布抓取流程进度

    ros::Subscriber sub_grab_pose = nh.subscribe("/wpb_home/grab_action", 1, GrabActionCallback);
    ros::Subscriber sub_beh = nh.subscribe("/wpb_home/behaviors", 30, BehaviorCB); // 随时准备获取抓取停止消息"grab stop"
    odom_ctrl_pub = nh.advertise<std_msgs::String>("/wpb_home/ctrl", 30);
    ros::Subscriber pose_diff_sub = nh.subscribe("/wpb_home/pose_diff", 1, PoseDiffCallback); // 获取机器人底盘电机码盘解算的里程计信息

    mani_ctrl_msg.name.resize(2);
    mani_ctrl_msg.position.resize(2);
    mani_ctrl_msg.velocity.resize(2);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.name[1] = "gripper";
    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
    mani_ctrl_msg.position[1] = 0.16;
    mani_ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)

    ros::NodeHandle nh_param("~");
    nh_param.getParam("grab/grab_y_offset", grab_y_offset);
    nh_param.getParam("grab/grab_lift_offset", grab_lift_offset);
    nh_param.getParam("grab/grab_forward_offset", grab_forward_offset);
    nh_param.getParam("grab/grab_gripper_value", grab_gripper_value);

    ros::Rate r(30);
    while(nh.ok())
    {
    
        //4、左右平移对准目标物品 
        if(nStep == STEP_OBJ_DIST)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2; //使用PID比例控制的方式控制机器人向前移动的速度，从而使机器人移动固定的距离到达距离物体fTargetGrabX的位置
            vy = (fMoveTargetY - pose_diff.y)/2; //使用PID比例控制的方式控制机器人左右平移的速度...

            VelCmd(vx,vy,0); //将vx和vy进行滤波（超过0.5的按照0.5算）然后发布

            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01) //当机器差不多移动到了指定位置的时候
            {
                VelCmd(0,0,0); //让机器人停下来
                ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;
                nStep = STEP_HAND_UP; //进入下一状态（抬起手臂）
            }

            result_msg.data = "object x";
            result_pub.publish(result_msg);
        }

        //5、抬起手臂
        if(nStep == STEP_HAND_UP)
        {
            if(nTimeDelayCounter == 0)
            {
                mani_ctrl_msg.position[0] = fObjGrabZ + grab_lift_offset; //发布机械臂抬升的信息，抬高的值等于物体的z轴坐标
                mani_ctrl_msg.position[1] = 0.16;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                ROS_WARN("[STEP_HAND_UP] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[1]);
                result_msg.data = "hand up";
                result_pub.publish(result_msg);
            }
            nTimeDelayCounter ++;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 20*30) //当过了挺长时间后， 机械臂肯定已经抬起来了 这时候准备进入下一个状态
            {
                fMoveTargetX = fTargetGrabX - 0.65 + grab_forward_offset; //0.65应该是机械臂展开后的长度 这个意思是更新需要前进的距离，让机器移动到机械臂正好能夹住物体的位置 ***要改
                fMoveTargetY = 0;
                ROS_WARN("[STEP_FORWARD] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
                nTimeDelayCounter = 0; 
                //ctrl_msg.data = "pose_diff reset";
                //odom_ctrl_pub.publish(ctrl_msg);
                nStep = STEP_FORWARD; //进入下一个状态（前进靠近物品）
            }
        }

         //6、前进靠近物品
        if(nStep == STEP_FORWARD)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2; //使用PID比例控制的方式控制机器人向前移动的速度，从而使机器人移动到机械臂正好能夹到物体的位置
            vy = (fMoveTargetY - pose_diff.y)/2; //这里其实因为上一个状态给的Y轴期望移动的距离为0，所以不平移

            VelCmd(vx,vy,0); //对刚才的速度进行滤波后发布

            //ROS_INFO("[STEP_FORWARD] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01) //当机器差不多移动到了机械臂正好能夹到物体的距离时
            {
                VelCmd(0,0,0); //让底盘别再乱动了
                ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_GRAB] grab_gripper_value = %.2f",grab_gripper_value);
                nStep = STEP_GRAB; //进入下一个状态（抓取物品）
            }

            result_msg.data = "forward";
            result_pub.publish(result_msg);
        }

        //7、抓取物品
        if(nStep == STEP_GRAB)
        {
            if(nTimeDelayCounter == 0)
            {
                result_msg.data = "grab";
                result_pub.publish(result_msg);
            }
            mani_ctrl_msg.position[1] = grab_gripper_value;      //抓取物品手爪闭合宽度
            mani_ctrl_pub.publish(mani_ctrl_msg);
            //ROS_WARN("[STEP_GRAB] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[1]);

            nTimeDelayCounter++;
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 8*30) //当经过足够长的时间后，爪子肯定已经闭合了
            {
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_OBJ_UP]");
                nStep = STEP_OBJ_UP; //进入下一个状态（拿起物品）
            }
        }

        //8、拿起物品
        if(nStep == STEP_OBJ_UP)
        {
            if(nTimeDelayCounter == 0)
            {
                mani_ctrl_msg.position[0] += 0.03; //夹着物品往上抬起3厘米
                mani_ctrl_pub.publish(mani_ctrl_msg);
                //ROS_WARN("[MANI_CTRL] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[1]);
                result_msg.data = "object up";
                result_pub.publish(result_msg);
            }
            nTimeDelayCounter++;
            VelCmd(0,0,0);
            mani_ctrl_pub.publish(mani_ctrl_msg);
            if(nTimeDelayCounter > 3*30) //当经过足够长的时间，机器肯定已经把物体抬起来了
            {
                fMoveTargetX = -(fObjGrabX -0.65 + grab_forward_offset); //更新机器人期望移动的距离，让机器人退回刚才的位置
                fMoveTargetY = -(fObjGrabY + grab_y_offset); //同上
                ROS_WARN("[STEP_BACKWARD] x= %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);

                nTimeDelayCounter = 0;
                nStep = STEP_BACKWARD; //进入下一状态
            }
        }

        //9、带着物品后退
        if(nStep == STEP_BACKWARD)
        {
            //ROS_WARN("[STEP_BACKWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
            //nTimeDelayCounter++;
            
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2; //使用PID比例控制的方式控制机器人向前移动的速度，从而让机器人退回刚才的位置

            if(pose_diff.x > -0.05) //如果机器人差不多退到之前的位置了
                vy = 0; //让机器人横向别乱动
            else
                vy = (fMoveTargetY - pose_diff.y)/2; //让机器人也调整横向的距离

            VelCmd(vx,vy,0); //将速度信息滤波后发布

            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01) //当机器人差不多退回了指定的位置
            {
                VelCmd(0,0,0); //让机器人速度清零并发布
                ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_DONE]");
                nStep = STEP_DONE; //进入下一个状态（抓取任务完毕）
            }

            result_msg.data = "backward";
            result_pub.publish(result_msg);
        }

        //10、抓取任务完毕
        if(nStep == STEP_DONE)
        {
            if(nTimeDelayCounter < 10)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter ++;
                result_msg.data = "done";
                result_pub.publish(result_msg);
            }
            else
            {
                nStep = STEP_WAIT;
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
