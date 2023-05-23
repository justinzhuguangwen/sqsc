// @brief  转身
// @brief  转身
// @date   2021年5月17日
// @auther justinzhu

#pragma once
#include "move_def.pc.h"
#include "move_key.pb.h"
#include "scene_obj_component_base.h"

class ACombatActor;

class RotateMove {
 public:
  RotateMove();
  ~RotateMove() = default;

  // @brief 设置转身目标方向
  // @param[in] dir 目标方向
  // @param[in] reason 转身原因
  // @param[in] reason_para 转身原因参数
  int SetDesiredDir(VECTOR3F dir, hx2proto::MoveReason reason = MoveReaon_None,
                    int64_t reason_para = 0);

  // @brief 更新转身
  // @param[in] actor
  // @param[in] diff_ms
  void UpdateRotate(ACombatActor *actor, uint64_t &diff_ms);

  // 大型物体转向接口-客户端请求
  int RotateImmediate(int reason, Vector3F desired_dir, uint64_t target_time);
  // svr有时间的旋转，逐步旋转
  // 是否已经转向到目标朝向
  bool IsRotating(ACombatActor *actor) const;
  void StopRotate();
  // 通知怪物转头
  static void NotifyMonsterRotateHead(uint64_t obj_id);
  // 延迟转身
  int DelayRotate(VECTOR3F tardir, float min_time, float speed, int64_t delay_ms, int reason,
                  uint32_t src_skillctx_instid);
  int RotateStep(VECTOR3F target_dir, NAVI_REASON reason, float min_time, float speed,
                 uint32_t skillinstid);

  // @brief 计算转身时间
  // @param[in] actor
  // @param[in] degree
  // @param[in] min_time
  // @param[in] speed
  // @return 转身时间，单位毫秒
  int64_t CalcRotateTime(ACombatActor *actor, int &degree, float min_time, float speed) const;

 public:
  ROTATEMOVE data_;
};