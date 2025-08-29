// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for custom_interfaces/SyncData
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4100)
#pragma warning(disable : 4265)
#pragma warning(disable : 4456)
#pragma warning(disable : 4458)
#pragma warning(disable : 4946)
#pragma warning(disable : 4244)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/sync_data.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class CUSTOM_INTERFACES_EXPORT ros2_custom_interfaces_msg_SyncData_common : public MATLABROS2MsgInterface<custom_interfaces::msg::SyncData> {
  public:
    virtual ~ros2_custom_interfaces_msg_SyncData_common(){}
    virtual void copy_from_struct(custom_interfaces::msg::SyncData* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const custom_interfaces::msg::SyncData* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_custom_interfaces_msg_SyncData_common::copy_from_struct(custom_interfaces::msg::SyncData* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //stamp
        const matlab::data::StructArray stamp_arr = arr["stamp"];
        auto msgClassPtr_stamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
        msgClassPtr_stamp->copy_from_struct(&msg->stamp,stamp_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'stamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'stamp' is wrong type; expected a struct.");
    }
    try {
        //ee_pose
        const matlab::data::StructArray ee_pose_arr = arr["ee_pose"];
        auto msgClassPtr_ee_pose = getCommonObject<geometry_msgs::msg::PoseStamped>("ros2_geometry_msgs_msg_PoseStamped_common",loader);
        msgClassPtr_ee_pose->copy_from_struct(&msg->ee_pose,ee_pose_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ee_pose' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ee_pose' is wrong type; expected a struct.");
    }
    try {
        //measurements
        const matlab::data::StructArray measurements_arr = arr["measurements"];
        auto msgClassPtr_measurements = getCommonObject<std_msgs::msg::Float32MultiArray>("ros2_std_msgs_msg_Float32MultiArray_common",loader);
        msgClassPtr_measurements->copy_from_struct(&msg->measurements,measurements_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'measurements' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'measurements' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_custom_interfaces_msg_SyncData_common::get_arr(MDFactory_T& factory, const custom_interfaces::msg::SyncData* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","stamp","ee_pose","measurements"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("custom_interfaces/SyncData");
    // stamp
    auto currentElement_stamp = (msg + ctr)->stamp;
    auto msgClassPtr_stamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
    outArray[ctr]["stamp"] = msgClassPtr_stamp->get_arr(factory, &currentElement_stamp, loader);
    // ee_pose
    auto currentElement_ee_pose = (msg + ctr)->ee_pose;
    auto msgClassPtr_ee_pose = getCommonObject<geometry_msgs::msg::PoseStamped>("ros2_geometry_msgs_msg_PoseStamped_common",loader);
    outArray[ctr]["ee_pose"] = msgClassPtr_ee_pose->get_arr(factory, &currentElement_ee_pose, loader);
    // measurements
    auto currentElement_measurements = (msg + ctr)->measurements;
    auto msgClassPtr_measurements = getCommonObject<std_msgs::msg::Float32MultiArray>("ros2_std_msgs_msg_Float32MultiArray_common",loader);
    outArray[ctr]["measurements"] = msgClassPtr_measurements->get_arr(factory, &currentElement_measurements, loader);
    }
    return std::move(outArray);
  } 
class CUSTOM_INTERFACES_EXPORT ros2_custom_interfaces_SyncData_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_custom_interfaces_SyncData_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_custom_interfaces_SyncData_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<custom_interfaces::msg::SyncData,ros2_custom_interfaces_msg_SyncData_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_custom_interfaces_SyncData_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<custom_interfaces::msg::SyncData,ros2_custom_interfaces_msg_SyncData_common>>();
  }
  std::shared_ptr<void> ros2_custom_interfaces_SyncData_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<custom_interfaces::msg::SyncData>();
    ros2_custom_interfaces_msg_SyncData_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_custom_interfaces_SyncData_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_custom_interfaces_msg_SyncData_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (custom_interfaces::msg::SyncData*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_custom_interfaces_msg_SyncData_common, MATLABROS2MsgInterface<custom_interfaces::msg::SyncData>)
CLASS_LOADER_REGISTER_CLASS(ros2_custom_interfaces_SyncData_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER