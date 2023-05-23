// codecc
/*
 * @brief 战斗体的移动部件
 * 包括：
 * 1. 移动的属性的读写接口
 *
 * @author walterwwang
 * @date 2021年2月22日
 */

#pragma once

#include <cstdint>
#include "component_meta.h"
#include "google/protobuf/stubs/port.h"
#include "lib_time.h"
#include "move_def.pb.h"
#include "move_def.pc.h"
#include "move_key.pb.h"
#include "nav_svc.h"
#include "physx_helper.h"
#include "pos_def.pb.h"
#include "px_user_data_base.h"
#include "scene_obj_component_base.h"
#include "star_comm.pc.h"
#include "star_cs.pb.h"
#include "star_key.pb.h"
#include "vector3.h"

namespace hx2proto {
class CSMoveInfo;
}

class CMoveComponent : public CombatActorComponentBase {
 public:
  CMoveComponent();
  ~CMoveComponent() override = default;
  const char *ClassName() const override { return "CMoveComponent"; }
  OBJ_PART_GET_TYPE_FUNC(hx2proto::SceneObjComType_Move);
  int CrossSerial(hx2proto::SSObjectCrossObjData &request) override;
  int CrossUnserial(const hx2proto::SSObjectCrossObjData &msg_real_data) override;
  bool IsGhostNeedCrossUnserial() const override { return true; }
  void OnEvent(const EVENTCONTEXT &evt) override;
  void Tick500ms() override;
  void Tick1s() override;
  friend class CCoAttrMng;
  friend class CPosComponent;

 public:
  //////////////////////////// 基础数据接口
  const MOVEINFO &Data() const { return data_; }
  int GetMapResID() const;
  VECTOR3F GetPos() const;
  VECTOR3F GetTarDir() const { return data_.target_dir; }
  VECTOR3F GetTarDir2D() const {
    return v3f::Normalized(VECTOR3F(data_.target_dir.x, data_.target_dir.y, 0));
  }
  VECTOR3F GetInputDir() const { return data_.input_dir; }
  int GetPathPointCount() const { return data_.path_point_list_count; }
  int SetPath(vector<POINTNODE> &path);
  void SetInputDir(VECTOR3F dir) { data_.input_dir = v3f::Normalized(dir); }
  void SetMoveReason(MoveReason reason) { data_.reason = reason; }
  // 将Navmesh寻路得到的路径设置到Actor上
  int SetNavmeshPath(vector<VECTOR3F> &path, int coord_type, int duration = 0,
                     VECTOR3F *ref_point = nullptr, VECTOR3F *face_point = nullptr,
                     MOVE_TYPE move_state = MOVE_TYPE_WALK);
  int SetSimplePath(VECTOR3F start, VECTOR3F end, int move_state = MOVE_TYPE_WALK);
  bool IsMoving() const;
  void ClearPathList();
  int64_t GetFutureStopTime() const { return data_.future_stop_time; }
  int GetCurState() const { return data_.cur_state; }
  void SetCurState(int state);
  // 当前状态变化后置处理
  void OnCurStateChanged(MOVE_TYPE old_state, MOVE_TYPE new_state);
  int GetSubState() const { return data_.sub_state; }
  // 设置子状态
  void SetSubState(int state);
  // 子状态变化后置处理
  void OnSubStateChanged(int old_state, int new_state);
  void SetAirMoveBeginTime(int64_t time) { data_.air_move_begin_time = time; }
  int64_t GetAirMoveBeginTime() const { return data_.air_move_begin_time; }
  VECTOR3F GetLastPos() const;
  // 体力值修改监听
  void OnAttrChange(hx2proto::ATTR_OBJ attr_index, int64_t p1, int64_t p2, int64_t newvalue,
                    int64_t change) override;

  //////////////////////////// 业务接口
  // 判断一个点是否在当前路径的目标终点附近（输入点会在函数内部贴地）
  bool IsPointNearDestPoint(VECTOR3F point, float near_dis = 100.0);
  // 获取寻路路径的指定索引的坐标 idx: [0, n] 0表示终点的索引, n表示起点的索引
  VECTOR3F GetDestPoint(int idx = 0);
  // 大型物体转向接口-客户端请求
  int RotateImmediate(int reason, Vector3F target_dir, uint64_t target_time);
  // svr有时间的旋转，逐步旋转
  // 是否已经转向到目标朝向
  bool IsRotating() const;
  void StopRotate();
  // 通知怪物转头
  void NotifyMonsterRotateHead(uint64_t obj_id);
  // 延迟转身
  int DelayRotate(VECTOR3F tardir, float min_time, float speed, int64_t delay_ms, int reason,
                  uint32_t src_skillctx_instid);
  int RotateStep(VECTOR3F target_dir, NAVI_REASON reason, float min_time, float speed,
                 uint32_t skillinstid);
  int Stop(StopMoveReason reason, VECTOR3F *newpos = nullptr, VECTOR3F *newdir = nullptr,
           UpdatePosReason posreason = hx2proto::kUpdatePosReason_None);
  // 广播移动数据
  int BroadcastMsg(BroadcastReason reason = BroadcastReason_None, bool include_self = false,
                   bool is_moving = true);
  // 广播速度变化
  void BroadcastSpeedChange();
  int OnPath(int time_in_advance = 0);
  // 延迟启动到转身等逻辑
  int TickDelay();
  void OnTick1sec();
  bool IsStatus(int commstat_type) const;

  // 设置靶向移动目标对象id
  void SetTargetObjID(uint64_t obj_id);
  // 获取靶向移动目标对象id
  uint64_t GetTargetObjID() const { return data_.target_obj_id; }
  // 是否靶向移动中
  bool IsTargetMove() const { return data_.target_obj_id > 0; }

  // 体力扣除
  void ConsumeEnergy(int energy);
  // 能量耗尽
  void OnEnergyExhaust();
  // 设置是否在草地上
  int SetInGrass(bool is_in);

 public:
  // 全量同步当前移动数据
  void SyncMapInfo(bool is_transfer = false);
  // 填充移动数据
  void FillCsMoveComInfo(CSMoveInfo &out, bool is_moving = true);
  // 是否由后台自动寻路和移动
  bool IsServerPathfinding();
  std::string DebugString() const override;
  int OnFinishPath();

  // fly
 public:
  bool IsFlying() const;
  AI_FLY_STATE GetCurFlyState() const { return data_.fly_state; }
  int FlyTakeOff(int height);
  int FlyCruise(VECTOR3F pos);
  int FlyLanding();
  int FlyPursuit(VECTOR3F pos);
  int OnStopMoveInAir();
  int FlyDie();

 public:
  uint32 GetFloatInstanceID() const { return data_.float_instance_id; }
  void SetFloatInstanceID(uint64_t intance_id) { data_.float_instance_id = intance_id; }
  // 更新 grid actor (pos_mark)
  // @param dest_point 目标点，举例：寻路路径有N个点，dest_point为最后一个点
  void UpdateActorPosMark(const VECTOR3F *dest_point = nullptr);

 private:
  // 停止后的逻辑
  int OnStop(StopMoveReason reason);
  float GetRotateTimeMs(int &degree, float min_time, float speed) const;
  // 生成路径的耗时
  void GenCostTime(float *dist = nullptr, float *time = nullptr, float rotate_min_time = 0,
                   float rotate_speed = 0, float *output_rotate_time = nullptr);
  void DoOnPath(uint64_t diff_ms, float speed, OUT VECTOR3F &pos);
  void OnTickRotate(uint64_t &diff_ms);
  int GetEntityType();
  void ModifyPath(vector<VECTOR3F> &path, VECTOR3F *ref_point);
  int SetTarDirData(VECTOR3F tar, SetDirReason reason = SetDirReason_None);
  // face_point 靶向移动面向的朝向目标点，为空则不使用靶向移动
  void FillPointDir(VECTOR3F *face_point = nullptr);
  // 修正路径上点太高的问题
  void FixPathPointListHeight(vector<VECTOR3F> &path);
  void RefillInnerPointForHeight(VECTOR3F a, VECTOR3F b, std::vector<VECTOR3F> &out);
  // 事件处理
  // 实体公共状态变化监听
  void OnEntityCommstatChanged(const EVENTCONTEXT &evt);
  // 视野变化监听
  void OnVisionUpdate(const EVENTCONTEXT &evt);

  // 是否满足潜行条件
  bool CanStartSneaking();
  // 尝试进入潜行状态
  bool TryStartSneaking();
  // 退出潜行状态
  void StopSneaking();
  // 是否潜行中
  bool IsSneaking();
  // 是否在草地内
  bool IsInGrass();

 private:
  MOVEINFO data_;
};
