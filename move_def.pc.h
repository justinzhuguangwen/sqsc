//Generated by StaticMemCppCodeByPb.py in hx2. DO NOT EDIT!  
//brief : only process Message in *.proto file, gen static memory cpp struct : 
//srcFile : modules/move/move_def.proto
//genDate : 2023-05-19 17:56:54


#ifndef _STATIC_MEM_CPP_STRUCT_FOR_PROTOBUF_MODULES_MOVE_MOVE_DEF_
#define _STATIC_MEM_CPP_STRUCT_FOR_PROTOBUF_MODULES_MOVE_MOVE_DEF_

#include <map>
#include <string>
#include "star_meta_info_comm.h"
#include "modules/move/move_def.pb.h"
using ::google::protobuf::int32;
using ::google::protobuf::uint32;
using ::google::protobuf::int64;
using ::google::protobuf::uint64;
using namespace hx2proto;
#ifndef COUNTOF
  #define COUNTOF(x) ((int)(sizeof(x)/sizeof(x[0])))
#endif

#include "comm/star_comm.pc.h"
//----------DELAYROTATE--------
// 延期转身，来自技能的转身节点
struct DELAYROTATE
{
    //--member list--
    int64 trigger_time; // delay anim
    VECTOR3F target_dir; // 
    float speed; // 
    float min_time; // 
    uint32 src_skill_instance_id; // 
    int32 reason; // 
    
    //--constructor/clear--
    DELAYROTATE();
    void Clear();
    DELAYROTATE(const int64& trigger_time__, const VECTOR3F& target_dir__, const float& speed__, const float& min_time__, const uint32& src_skill_instance_id__, const int32& reason__);
    
    //--function for transfer to/from pb--
    int CopyFromPb(const DelayRotate& in);
    int CopyToPb(DelayRotate& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
};
typedef DELAYROTATE* LPDELAYROTATE;
typedef DELAYROTATE tagDelayRotate;


//----------MOVEINFO--------
// 
struct MOVEINFO
{
    //--member list--
    int32 cur_state; // 当前状态 MOVE_TYPE
    int32 sub_state; // 子状态 MoveSubType
    VECTOR3F input_dir; // 客户端输入朝向，校验用，不广播
    int32 path_point_list_count;
    POINTNODE path_point_list[40]; // [max_count:40] 路径
    VECTOR3F target_dir; // 目标朝向，已归一化
    int64 future_stop_time; // 怪物预计走完路程的时间
    int64 last_rotate_time; // 上次原地旋转的时间戳
    uint32 rotate_src_skill_id; // 触发转身的skillinstid
    DELAYROTATE delay_rotate; // 延迟转身
    MoveReason reason; // 移动原因
    uint32 float_instance_id; // 造成浮空的 InstanceID
    AI_FLY_STATE fly_state; // 飞行状态
    uint64 target_obj_id; // 靶向移动目标ID，此字段不为0时优先使用此目标的方向覆盖dir方向
    int32 mistake_dist; // 误差累计距离(用于反外挂,累计时间内误差距离过大认为存在作弊)
    int32 mistake_time; // 误差累计时间
    int64 air_move_begin_time; // 上次浮空位移开始时间
    int64 last_energy_consume_time; // 上次消耗体力时间
    int64 last_energy_recover_tick; // 上次体力恢复tick
    bool is_sneaking; // 是否在潜行状态
    bool is_in_grass; // 是否在草地上
    
    //--constructor/clear--
    MOVEINFO();
    void Clear();
    
    //--function for transfer to/from pb--
    int CopyFromPb(const MoveInfo& in);
    int CopyToPb(MoveInfo& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
    bool is_path_point_list_full() const { return path_point_list_count >= 40; }
    bool add_path_point_list(const POINTNODE &r);
    bool remove_path_point_list(int index);
};
typedef MOVEINFO* LPMOVEINFO;
typedef MOVEINFO tagMoveInfo;


//----------TARGETMOVEMENT--------
// 靶向移动
struct TARGETMOVEMENT
{
    //--member list--
    uint64 look_at_target_obj_id; // 面向目标ID
    uint64 target_obj_id; // 靶向移动目标ID，此字段不为0时优先使用此目标的方向覆盖dir方向
    
    //--constructor/clear--
    TARGETMOVEMENT();
    void Clear();
    TARGETMOVEMENT(const uint64& look_at_target_obj_id__, const uint64& target_obj_id__);
    
    //--function for transfer to/from pb--
    int CopyFromPb(const TargetMovement& in);
    int CopyToPb(TargetMovement& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
};
typedef TARGETMOVEMENT* LPTARGETMOVEMENT;
typedef TARGETMOVEMENT tagTargetMovement;


//----------ROTATEMOVE--------
// 转向移动
struct ROTATEMOVE
{
    //--member list--
    int64 last_rotate_time; // 上次原地旋转的时间戳
    DELAYROTATE delay_rotate; // 延迟转身
    VECTOR3F desired_dir; // 目标朝向，已归一化 direction
    
    //--constructor/clear--
    ROTATEMOVE();
    void Clear();
    ROTATEMOVE(const int64& last_rotate_time__, const DELAYROTATE& delay_rotate__, const VECTOR3F& desired_dir__);
    
    //--function for transfer to/from pb--
    int CopyFromPb(const RotateMove& in);
    int CopyToPb(RotateMove& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
};
typedef ROTATEMOVE* LPROTATEMOVE;
typedef ROTATEMOVE tagRotateMove;


//----------MOVEANTICHEAT--------
// 反外挂相关
struct MOVEANTICHEAT
{
    //--member list--
    int32 mistake_dist; // 误差累计距离(用于反外挂,累计时间内误差距离过大认为存在作弊)
    int32 mistake_time; // 误差累计时间
    
    //--constructor/clear--
    MOVEANTICHEAT();
    void Clear();
    MOVEANTICHEAT(const int32& mistake_dist__, const int32& mistake_time__);
    
    //--function for transfer to/from pb--
    int CopyFromPb(const MoveAntiCheat& in);
    int CopyToPb(MoveAntiCheat& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
};
typedef MOVEANTICHEAT* LPMOVEANTICHEAT;
typedef MOVEANTICHEAT tagMoveAntiCheat;


//----------MOVEENERGY--------
// 体力相关数据
struct MOVEENERGY
{
    //--member list--
    int64 last_energy_consume_time; // 上次消耗体力时间
    int64 last_energy_recover_tick; // 上次体力恢复tick
    
    //--constructor/clear--
    MOVEENERGY();
    void Clear();
    MOVEENERGY(const int64& last_energy_consume_time__, const int64& last_energy_recover_tick__);
    
    //--function for transfer to/from pb--
    int CopyFromPb(const MoveEnergy& in);
    int CopyToPb(MoveEnergy& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
};
typedef MOVEENERGY* LPMOVEENERGY;
typedef MOVEENERGY tagMoveEnergy;


//----------SNEAKMOVE--------
// 潜行
struct SNEAKMOVE
{
    //--member list--
    bool is_sneaking; // 是否在潜行状态
    bool is_in_grass; // 是否在草地上
    
    //--constructor/clear--
    SNEAKMOVE();
    void Clear();
    SNEAKMOVE(const bool& is_sneaking__, const bool& is_in_grass__);
    
    //--function for transfer to/from pb--
    int CopyFromPb(const SneakMove& in);
    int CopyToPb(SneakMove& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
};
typedef SNEAKMOVE* LPSNEAKMOVE;
typedef SNEAKMOVE tagSneakMove;


//----------MOVESOURCEINFO--------
// 移动的触发来源
struct MOVESOURCEINFO
{
    //--member list--
    MoveReason reason; // 移动原因
    uint32 skill_inst_id; // 技能ID  原： uint32 RotateSrcSkillID = 2;
    
    //--constructor/clear--
    MOVESOURCEINFO();
    void Clear();
    MOVESOURCEINFO(const MoveReason& reason__, const uint32& skill_inst_id__);
    
    //--function for transfer to/from pb--
    int CopyFromPb(const MoveSourceInfo& in);
    int CopyToPb(MoveSourceInfo& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
};
typedef MOVESOURCEINFO* LPMOVESOURCEINFO;
typedef MOVESOURCEINFO tagMoveSourceInfo;


//----------MOVEPATHDATA--------
// 
struct MOVEPATHDATA
{
    //--member list--
    int32 point_list_count;
    POINTNODE point_list[40]; // [max_count:40] 路径
    int64 estimated_stop_time; // 怪物预计走完路程的时间戳
    
    //--constructor/clear--
    MOVEPATHDATA();
    void Clear();
    
    //--function for transfer to/from pb--
    int CopyFromPb(const MovePathData& in);
    int CopyToPb(MovePathData& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
    bool is_point_list_full() const { return point_list_count >= 40; }
    bool add_point_list(const POINTNODE &r);
    bool remove_point_list(int index);
};
typedef MOVEPATHDATA* LPMOVEPATHDATA;
typedef MOVEPATHDATA tagMovePathData;


//----------MOVEDATA--------
// 
struct MOVEDATA
{
    //--member list--
    MoveModeDef move_mode; // 移动模式
    MoveStateDef move_state; // 移动状态
    MOVESOURCEINFO source; // 移动来源
    TARGETMOVEMENT target_move; // 靶向移动
    ROTATEMOVE rotate_move; // 转向移动
    MOVEANTICHEAT anti_cheat; // 反外挂相关
    MOVEENERGY energy; // 体力相关数据
    SNEAKMOVE sneak_move; // 潜行
    uint32 float_instance_id; // 造成浮空的 InstanceID
    int64 air_move_begin_time; // 上次浮空位移开始时间
    VECTOR3F input_dir; // 客户端输入朝向，校验用，不广播
    
    //--constructor/clear--
    MOVEDATA();
    void Clear();
    
    //--function for transfer to/from pb--
    int CopyFromPb(const MoveData& in);
    int CopyToPb(MoveData& out) const;
    bool SerializeToString(std::string *out) const; 
    bool ParseFromString(const std::string &in); 
    std::string DebugString() const; 
    std::string Debug2Json() const; 
};
typedef MOVEDATA* LPMOVEDATA;
typedef MOVEDATA tagMoveData;


#endif