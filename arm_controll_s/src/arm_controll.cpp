#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyS2"

#define MOVEMENT_UPDATE_RATE                 10

DynamixelWorkbench dxl_wb;

class DynamixelMethods{
    private:
        uint16_t model_number = 0;
    protected:
        bool dxlInit(){
            bool result = false;
            const char *log;
            result = dxl_wb.init(DEVICENAME, BAUDRATE, &log);
            if (result == false){
                ROS_ERROR_STREAM(log);
                ROS_ERROR_STREAM("Failed to init");
                return false;
            }
            else{
                ROS_INFO_STREAM("Succeed to init: "<<BAUDRATE); 
            }
            return(true);
        }

        int32_t readPosition(uint8_t id){
            const char*log;
            bool result = false;
            int32_t get_data = 0;
            result = dxl_wb.itemRead(id, "Present_Position", &get_data, &log);
            
            if (result == false)
            {
                return (int32_t)-1;
            }
            return get_data;
        }

        int32_t readVelocity(uint8_t id){
            const char*log;
            bool result = false;
            int32_t get_data = 0;
            result = dxl_wb.itemRead(id, "Present_Speed", &get_data, &log);
            
            if (result == false)
            {
                return (int32_t)-1;
            }
            return get_data;
        }

        bool pingMotor(uint8_t id){
            const char *log;
            bool result = false;
            result = dxl_wb.ping(id,&model_number,&log);
            if (result == false){
                ROS_ERROR_STREAM(log);
                ROS_ERROR_STREAM("Failed to ping id: "<<(int)id);
            }
            return result;
        }

        bool setJointMode(uint8_t id, int32_t velocity){
            const char *log;
            bool result = false;
            result = dxl_wb.jointMode(id, velocity, 100, &log); //first value - speed, second value - acceleration
            if (!result){
                ROS_ERROR_STREAM(log);
                ROS_ERROR_STREAM("Failed to change joint mode");
                return false;
            }
            return true;
        }

        bool setPose(uint8_t id ,int32_t val){
            bool result = false;
            const char*log;
            result = dxl_wb.writeRegister(id, "Goal_Position", val, &log);
            if(!result) ROS_ERROR_STREAM("Cant set position id: "<<(int)id);
            return result;
        }
};

class Manipulator: public DynamixelMethods{
    public:
        ros::Subscriber joint_state_subscriber;
        int DXL_COUNT;
        ros::Subscriber torque_subscriber;

        Manipulator(){}

        Manipulator(int  DXL_COUNT_, int * dxl_id_, float * present_position_, float * present_velocity_, ros::NodeHandle * nh_){
            DXL_COUNT = DXL_COUNT_;
            dxl_id = new uint8_t[DXL_COUNT];
            goal_velocity = new int32_t[DXL_COUNT];
            goal_position = new int32_t[DXL_COUNT];
            present_velocity = new float[DXL_COUNT];
            present_position = new float[DXL_COUNT];
            nh = nh_;
            joint_state_publisher = nh->advertise<sensor_msgs::JointState>("/joint_states", 64);

            for(int i=0; i!=DXL_COUNT; i++){
                present_position[i] = present_position_[i];
                present_velocity[i] = present_velocity_[i];
                goal_position[i] = present_position_[i];
                goal_velocity[i] = present_velocity_[i];
                dxl_id[i] = (uint8_t)dxl_id_[i];
                disable_flag = false;
                joint_state_msg.position.resize(DXL_COUNT);
                joint_state_msg.velocity.resize(DXL_COUNT);
                joint_state_msg.effort.resize(DXL_COUNT);
            }

            if (!dxlInit()){
                ROS_ERROR_STREAM("Cant initialize dxl manipualtor");
                exit(1);
            }
            if (!pingAllMotors()){
                ROS_ERROR_STREAM("Cant ping dxl manipualtor");
                exit(1);
            }
            
            if (!setAllJointMode()){
                ROS_ERROR_STREAM("Cant set joint mode for manipulator");
                exit(1);
            }

            addHandlersAndRegulators();
            ROS_INFO_STREAM("Manipulator have been initialized successfully");
        }

        bool motorTorque(bool flag){
            const char*log;
            bool result = false;
            for(int i=0; i != DXL_COUNT; i++){
                if (flag){
                    result = dxl_wb.torqueOn(dxl_id[i], &log);
                }
                else{
                    result = dxl_wb.torqueOff(dxl_id[i], &log);
                }
                if(result == false){
                    ROS_ERROR_STREAM(log);
                    return false;
                }
                else{
                    ROS_INFO_STREAM("Motor torque change: "<<(int)(i+1));
                }
            }
            return true;
        }

        bool updateCurrentVelPos(){
            int32_t data;
            for(int i = 0; i != DXL_COUNT; i++){
                data = readPosition(dxl_id[i]);
                if (data == -1){
                    return false;
                }
                present_position[i] = dxl_wb.convertValue2Radian(dxl_id[i], data);
                data = readVelocity(dxl_id[i]);
                if (data == -1){
                    return false;
                }
                present_velocity[i] = dxl_wb.convertValue2Velocity(dxl_id[i], data); 
            }
            return true;
        }

        void publishCurrentJointState(){
            for(int i = 0; i != DXL_COUNT; i++){
                joint_state_msg.position[i] = present_position[i];
                joint_state_msg.velocity[i] = present_velocity[i];
            }
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_publisher.publish(joint_state_msg);
        }

        void messageCmdJointsCallback(const sensor_msgs::JointState::ConstPtr& toggle_msg) {
            if (toggle_msg->position.size() < DXL_COUNT){
                return;
            }
            for(int i = 0; i<DXL_COUNT; i++){
                goal_position[i] = dxl_wb.convertRadian2Value(dxl_id[i], toggle_msg->position[i]);
                goal_velocity[i] = dxl_wb.convertVelocity2Value(dxl_id[i], toggle_msg->velocity[i]);
            }
            updateGoalVelPos();
        }

        void messageTorqueCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
            if(msg->data) motorTorque(true);
            else motorTorque(false);
        }

        ~Manipulator(){
            delete [] dxl_id;
            delete [] goal_position;
            delete [] goal_velocity;
            delete [] present_position;
            delete [] present_velocity;
        }
        
    private:
        
        uint8_t *dxl_id;
        int32_t *goal_position;
        int32_t *goal_velocity;
        float *present_position;
        float *present_velocity;
        bool disable_flag;
        ros::NodeHandle * nh = 0;
        sensor_msgs::JointState joint_state_msg;
        ros::Publisher joint_state_publisher;

        bool pingAllMotors(){
            for(int i = 0; i < DXL_COUNT; i++){
                if (!pingMotor(dxl_id[i])){
                    return false;
                }
                else{
                    ROS_INFO_STREAM("Ping motor: "<<(i+1));
                }
            }
            return true;
        }

        bool setAllJointMode(){

            for (int i = 0; i< DXL_COUNT; i++){
                if(!setJointMode(dxl_id[i], goal_velocity[i])) return false;
            }
            return true;
        }

        void addHandlersAndRegulators(){
            const char *log;
            bool result = false;
            for(int cnt=0; cnt<DXL_COUNT; cnt++){
                result = dxl_wb.writeRegister(dxl_id[cnt], "CCW_Compliance_Slope", 30, &log);
                result = dxl_wb.writeRegister(dxl_id[cnt], "CW_Compliance_Slope", 30, &log);
                // ROS_INFO_STREAM(result);
            }
            
            dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position",&log);
            // ROS_INFO_STREAM(log);
            dxl_wb.addSyncWriteHandler(dxl_id[0], "Moving_Speed",&log);
            // ROS_INFO_STREAM(log);
        }

        bool updateGoalVelPos(){
            bool result = false;
            const char*log;
            for(int i=0; i<DXL_COUNT; i++){
                result = dxl_wb.writeRegister(dxl_id[i], "Goal_Position", goal_position[i], &log);
                result = dxl_wb.writeRegister(dxl_id[i], "Moving_Speed", goal_velocity[i], &log);
                if (!result){
                    ROS_ERROR_STREAM(log<<" id: "<<(int)(i+1));
                }
            }
            return result;
        }
};

class Gripper: public DynamixelMethods{
    public:
        ros::Subscriber gripper_subscriber;

        Gripper(){}
        Gripper(int dxl_gripper_id, int velocity, ros::NodeHandle * nh_){
            dxl_id = (uint8_t)dxl_gripper_id;
            nh = nh_;
            if (!dxlInit()){
                ROS_ERROR_STREAM("Cant initialize dxl manipualtor");
                exit(1);
            }
            if (!pingMotor(dxl_id)){
                ROS_ERROR_STREAM("Cant ping dxl manipualtor");
                exit(1);
            }
            else ROS_INFO_STREAM("Ping gripper motor. Id: "<<(int)dxl_id);
            
            if (!setJointMode(dxl_id, (int32_t)velocity)){
                ROS_ERROR_STREAM("Cant set joint mode for manipulator");
                exit(1);
            }
            ROS_INFO_STREAM("Gripper have been initialized successfully");
        }
    
        void messageGripperCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
            if(msg->data) open();
            else close();
        }

        void open(){
            if (!state || state == NULL){
                setPose(dxl_id, open_position);
                state = true;
            }
        }
        void close(){
            if (state || state == NULL){
                setPose(dxl_id, close_position);
                state = false;
            }
        }

    private:
        uint8_t dxl_id;
        int32_t open_position = 720;
        int32_t close_position = 512;
        bool state = NULL;
        ros::NodeHandle * nh;
};

Manipulator& setManipulator(std::string manipulator_type,ros::NodeHandle * nh){
    Manipulator * manipulator = 0;
    
    if(manipulator_type == "palletizer"){
        int DXL_COUNT = 4;
        int dxl_id[] = {1,2,3,4};
        float present_position[] = {0,0,0,0};
        float present_velocity[] = {40,40,40,40};
        manipulator = new Manipulator(DXL_COUNT, dxl_id, present_position, present_velocity, nh);
    }

    if(manipulator_type == "angle"){
        int DXL_COUNT = 5;
        int dxl_id[] = {1,2,3,4,5};
        float present_position[] = {0,0,0,0,0};
        float present_velocity[] = {40,40,40,40};
        manipulator = new Manipulator(DXL_COUNT, dxl_id, present_position, present_velocity, nh);
    }
    return *manipulator;
}

int main(int argc, char **argv){   
    ros::init(argc,argv, "Dxl_Arm");
    ros::NodeHandle nh;

    std::string manipulator_type;
    bool gripper_enable;
    int end_eff_id;

    std::vector<std::string> keys;
    nh.getParamNames(keys);

    for(int i=0; i< keys.size(); i++){
         if(keys[i].find("manipulator_type") != std::string::npos){
             ros::param::get(keys[i], manipulator_type);
         }
         if(keys[i].find("gripper") != std::string::npos){
             ros::param::get(keys[i], gripper_enable);
         }
         if(keys[i].find("end_eff_id") != std::string::npos){
             ros::param::get(keys[i], end_eff_id);
         }
    }        

    ros::param::get("manipulator_type", manipulator_type);
    ros::param::get("gripper", gripper_enable);
    ros::param::get("end_eff_id", end_eff_id);

    Manipulator manipulator;
    Gripper gripper;

    try{
        manipulator = setManipulator(manipulator_type, &nh);
    }
    catch (const char* exception){
        ROS_ERROR_STREAM("Cant find select manipulator type");
        return 1;
    }

    if (gripper_enable){
        gripper = Gripper(end_eff_id, 50, &nh);
        gripper.gripper_subscriber = nh.subscribe<std_msgs::Bool>("/end_eff_cmd", 10, &Gripper::messageGripperCmdCallback, &gripper);
    }
    // std::cout<<manipulator.DXL_COUNT<<std::endl;
    manipulator.joint_state_subscriber = nh.subscribe<sensor_msgs::JointState> ("/cmd_joints", 10, &Manipulator::messageCmdJointsCallback, &manipulator);
    manipulator.torque_subscriber =  nh.subscribe<std_msgs::Bool> ("/disable_torque", 10, &Manipulator::messageTorqueCmdCallback, &manipulator);

    ros::Rate loop_rate(MOVEMENT_UPDATE_RATE);
    while (ros::ok())
    {
        manipulator.updateCurrentVelPos();
        manipulator.publishCurrentJointState();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
