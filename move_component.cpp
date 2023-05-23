/*
 * @brief 战斗体的移动部件
 * 包括：
 * 1. 移动的属性的读写接口
 *
 * @author walterwwang
 * @date 2021年2月22日
 */

#include "move_component.h"

#include <vector>

#include "action_impl.h"
#include "combat_actor.h"
#include "combat_monster.h"
#include "combat_role.h"
#include "combat_vehicle.h"
#include "comm_event_def.pc.h"
#include "component_meta.h"
#include "dep/physx/pxshared/include/foundation/PxTransform.h"
#include "hx_log.h"
#include "lib_log.h"
#include "lib_math.h"
#include "lib_str.h"
#include "lib_time.h"
#include "lib_time_source.h"
#include "map.h"
#include "move_def.pc.h"
#include "move_key.pb.h"
#include "obj_attr_mng.h"
#include "object_factory.h"
#include "physx_helper.h"
#include "processor.h"
#include "px_scene_mgr.h"
#include "real_ghost_rpc.pb.h"
#include "scene_login_rpc.pb.h"
#include "scene_obj_component_base.h"
#include "star_gid.pb.h"
#include "star_key.pb.h"
#include "vector3.h"
#include "vision.h"
#include "vision_refresh_rpc.pb.h"

int g_view_path_time = 5;
static const char *MoveStateStr(int state) {
  if (state < MOVE_TYPE_MIN || state > MOVE_TYPE_MAX) {
    return "unknown";
  }
  return MOVE_TYPE_Name(static_cast<hx2proto::MOVE_TYPE>(state)).c_str();
}

COM_META_REGISTER(CMoveComponent, (TickInterList{TickInter::k500ms, TickInter::k1s}),
                  (EvtIDList{EVENTCONTEXT::kObjDie, EVENTCONTEXT::kSkillFinish,
                             EVENTCONTEXT::kPosMove, EVENTCONTEXT::kEntityCommstatChanged,
                             EVENTCONTEXT::kVisionUpadte, EVENTCONTEXT::kAddMemberTaunt,
                             EVENTCONTEXT::kTauntExistToEmpty}));

void CMoveComponent_RegisterMeta() {
  REG_COM_INDI_META(ATTR_MOVE_STATE, MOVEINFO, cur_state);
  REG_COM_INDI_META(ATTR_FLY_STATE, MOVEINFO, fly_state);
  REG_COM_INDI_META(ATTR_DYN_SNEAK, MOVEINFO, is_sneaking);
  REG_COM_INDI_META(ATTR_TARGET_OBJ_ID, MOVEINFO, target_obj_id);
}

CMoveComponent::CMoveComponent() {
  if (SHM_MODE_INIT == get_shm_mode()) {
    BZEROST(data_);
  } else {
  }
}

void CMoveComponent::OnEvent(const EVENTCONTEXT &evt) {
  // TODO(walterwwang) kDie process
  switch (evt.get_event_union_type()) {
    case EVENTCONTEXT::kSkillFinish: {
      const auto &param = evt.event.skill_finish;
      if (param.skill_ctx_instance_id == data_.rotate_src_skill_id ||
          param.skill_ctx_instance_id == data_.delay_rotate.src_skill_instance_id) {
        LogDebug("skill interrupt finish, stop rotate");
        Stop(StopMoveReason_SkillInterrupt);
      }
      // 浮空技能打断，当前是锁定角色的上下文，且是被打断，才中断
      if (param.is_lock_role_inst && param.skill_ctx_instance_id == data_.float_instance_id &&
          GetCoOwner()->GetAttr(ATTR_DYN_FLOAT) > 0) {
        LogDebug("float interrupt, inst:%d, reason:%s", param.skill_ctx_instance_id,
                 RemoveCombatCtxReason_Name(param.reason).c_str());
        GetCoOwner()->SetAttr(ATTR_DYN_FLOAT, 0);
        data_.float_instance_id = 0;
      }
      break;
    }
    case EVENTCONTEXT::kObjDie: {
      if (GetCurState() == MOVE_TYPE::MOVE_TYPE_FLY) {
        FlyDie();
      } else {
        Stop(StopMoveReason_Die);
      }
      break;
    }
    case EVENTCONTEXT::kPosMove: {
      const auto &move = evt.event.pos_move;
      // 瞬移时更新下占位
      if (move.reason == kUpdatePosReason_SkillFlashMove) {
        UpdateActorPosMark();
      }
      break;
    }
    case EVENTCONTEXT::kVisionUpadte: {  // 视野更新
      OnVisionUpdate(evt);
      break;
    }
    case EVENTCONTEXT::kAddMemberTaunt: {  // 增加仇恨
      if (GetOwnerType() != GID_ROLE)
        return;
      // 有仇恨列表，直接退出潜行
      StopSneaking();

      break;
    }
    case EVENTCONTEXT::kTauntExistToEmpty: {  // 仇恨变空
      if (GetOwnerType() != GID_ROLE)
        return;
      TryStartSneaking();
      break;
    }
    case EVENTCONTEXT::kEntityCommstatChanged: {  // 对象公共状态变化
      OnEntityCommstatChanged(evt);
      break;
    }
    default:
      break;
  }
}

// 体力扣除
void CMoveComponent::ConsumeEnergy(int energy) {
  // 参数过滤
  if (energy <= 0) {
    return;
  }

  // 修改体力值
  auto new_energy = MAX(0, GetOwner()->GetAttr(ATTR_CURENERGY) - energy);
  GetOwner()->SetAttr(ATTR_CURENERGY, new_energy);
}

// 能量耗尽
void CMoveComponent::OnEnergyExhaust() {
  auto cur_energy = GetOwner()->GetAttr(ATTR_CURENERGY);
  if (cur_energy > 0) {
    return;
  }

  // 状态修改
  int old_state = GetCurState();
  auto sub_state = GetSubState();
  // 能量为空的检查
  bool changed = false;
  if (cur_energy <= 0) {
    switch (old_state) {
      case MOVE_TYPE_WALK:
      case MOVE_TYPE_SWIM: {
        if (MoveSubType_Normal != sub_state) {
          data_.sub_state = MoveSubType_Normal;
        }
        changed = true;
      } break;
      case MOVE_TYPE_FLY:
      case MOVE_TYPE_GLIDE: {
        // 开始自由落体
        SetCurState(MOVE_TYPE_FREE_FALL);
        changed = true;
      } break;
      default:
        break;
    }
    if (changed) {
      LogDebugM(LOGM_MOVE, "co:%ld, old status(%s), new status(%s)", GetOwnerid(),
                MoveStateStr(old_state), MoveStateStr(GetCurState()));
      SyncMapInfo();
    }
  }
}

// 设置是否在草地上
int CMoveComponent::SetInGrass(bool is_in) {
  data_.is_in_grass = is_in;
  if (is_in) {  // 检查是否需要进入潜行
    TryStartSneaking();
  } else {      // 退出潜行
    StopSneaking();
  }
  return 0;
}

VECTOR3F CMoveComponent::GetLastPos() const {
  return data_.path_point_list_count > 0
             ? data_.path_point_list[data_.path_point_list_count - 1].pos
             : VECTOR3F();
}

// 体力值修改监听
void CMoveComponent::OnAttrChange(hx2proto::ATTR_OBJ attr_index, int64_t p1, int64_t p2,
                                  int64_t newvalue, int64_t change) {
  // 体力扣减
  if (attr_index == hx2proto::ATTR_CURENERGY && change < 0) {
    // 托管到坐骑组件的无需处理
    auto *mount_com = GetOwner()->GetCom<CMountComponent>();
    if (mount_com && mount_com->IsOnMount()) {
      return;
    }

    // 修改消耗体力时间
    data_.last_energy_consume_time = GetRealTimeMs();

    // 能量耗尽
    if (newvalue <= 0) {
      OnEnergyExhaust();
    }
  }
}

int CMoveComponent::SetTarDirData(VECTOR3F tar, SetDirReason reason) {
  auto tardir = v3f::Normalized(tar);

  if (IS_V3F_ZERO(tardir)) {
    tardir = VECTOR3F(1, 0, 0);
  }

  if (tardir == GetTarDir()) {
    return 0;
  }
  LogTraceM(LOGM_MOVE, "objid:%lu, dir:%s-->%s, reason:%d,%s, on vehicle(%d)", GetOwnerid(),
            V3FSTR(GetCoOwner()->PosCom().GetDir()), V3FSTR(tardir), reason,
            SetDirReason_Name(reason).c_str(), GetCoOwner()->IsOnVehicle());
  data_.target_dir = tardir;

  return 0;
}

void CMoveComponent::Tick500ms() {
  TickDelay();
  OnPath();
}

void CMoveComponent::Tick1s() { OnTick1sec(); }

int CMoveComponent::OnStop(StopMoveReason reason) {
  EVENTCONTEXT evt;
  auto event = evt.mutable_pos_stop();
  event->map_id = GetCoOwner()->GetMapResID();
  event->new_pos = GetPos();
  event->reason = reason;
  INIT_EVT_OBJ_SIMPLE(evt, GetVisiOwner());
  GetOwner()->OnEvent(evt);  // 同步事件
  return 0;
}

int CMoveComponent::SetPath(vector<POINTNODE> &path) {
  ClearPathList();
  for (int i = 0; i < static_cast<int>(path.size()); ++i) {
    data_.add_path_point_list(path[i]);
  }
  return 0;
}

int CMoveComponent::CrossSerial(SSObjectCrossObjData &request) {
  data_.CopyToPb(*request.mutable_movecom());
  return 0;
}

int CMoveComponent::CrossUnserial(const SSObjectCrossObjData &msg_real_data) {
  data_.CopyFromPb(msg_real_data.movecom());
  return 0;
}
int CMoveComponent::Stop(StopMoveReason reason, VECTOR3F *newpos, VECTOR3F *newdir,
                         hx2proto::UpdatePosReason posreason) {
  if (GetCurState() == MOVE_TYPE_STOP && !newpos && !newdir) {
    return 0;
  }

  LogDebugM(
      LOGM_MOVE,
      "obj stop move, id:%lu/%d, reason:%d,%s, leftCount:%d, state:%s, dir:%s, curpos:%s newpos:%s "
      "newdir:%s updatePosReason:%s",
      GetOwnerid(), GetCoOwner()->GetResid(), reason, StopMoveReason_Name(reason).c_str(),
      data_.path_point_list_count,
      MOVE_TYPE_Name(static_cast<hx2proto::MOVE_TYPE>(GetCurState())).c_str(),
      V3FSTR(GetCoOwner()->PosCom().GetDir()), V3FSTR(GetPos()), newpos ? V3FSTR(*newpos) : "",
      newdir ? V3FSTR(*newdir) : "", UpdatePosReason_Name(posreason).c_str());

  // 更新位置
  OnPath();
  ClearPathList();
  GetCoOwner()->NavCom().SetNavReason(NAVI_REASON_NONE);

  data_.sub_state = 0;
  auto &poscom = GetCoOwner()->PosCom();

  bool is_changed = false;
  if (nullptr != newpos && GetPos() != *newpos) {
    poscom.UpdatePos(*newpos, true, posreason);
    is_changed = true;
  }
  if (nullptr != newdir && GetCoOwner()->PosCom().GetDir() != *newdir) {
    poscom.SetDir(*newdir, SetDirReason_UpdatePos);
    is_changed = true;
  }
  StopRotate();
  auto *co = GetCoOwner();
  if (!co->IsPet()) {
    poscom.SetLastMoveTime(GetRealTickTimeMs());
  } else {
    poscom.SetLastMoveTime(0);
  }
  if (reason == StopMoveReason_SkillInterrupt) {
    data_.reason = MoveReason_SkillStop;
  }
  int iOldState = GetCurState();
  SetCurState(MOVE_TYPE_STOP);
  if (iOldState != MOVE_TYPE_STOP || is_changed) {
    BroadcastMsg(BroadcastReason_RoleStop, true);
  }
  OnStop(reason);

  // TODO(@henrycao) by alexxfxiao fix bug，这里怪物的skillstop后reason一直没改回来，
  // 导致后面的正常的下发的move消息里的reason也是skillstop，带来bug：客户端里怪物没有action动作
  // 这个reason应该仅用于给客户端发消息，本来不应该存储带来维护代价。先简单吧内存数据在发完消息后改回来
  if (data_.reason == MoveReason_SkillStop) {
    data_.reason = hx2proto::MoveReason_Normal;
  }

  // 移动停止, 需更新physx actor
  UpdateActorPosMark();
  return 0;
}

int CMoveComponent::BroadcastMsg(BroadcastReason reason, bool include_self, bool is_moving) {
  ACombatActor *pstCo = GetCoOwner();

  const auto &poscom = pstCo->PosCom();
  CSMsgVision stPkg;
  auto pstMsg = stPkg.mutable_movebroadcast();
  if (IsServerPathfinding()) {
    // 怪物补偿100ms RTT时间
    if (BroadcastReason_RoleStop == reason) {
      // 停止的时候，服务器驱动的角色，发送当前移动的时间戳
      pstMsg->set_timestamp(poscom.GetLastMoveTime() + kMonsterTimestampCompensation);
    } else {
      // 其他情况下的时间戳，使用当前路径的到达时间
      pstMsg->set_timestamp(GetFutureStopTime() + kMonsterTimestampCompensation);
    }
  } else {
    pstMsg->set_timestamp(poscom.GetLastMoveTime());
  }
  pstMsg->set_objid(GetOwnerid());
  GetCoOwner()->FillCsMoveInfo(*pstMsg->mutable_data(), is_moving);

  // debug，客户端显示移动路径，不依赖于ue，且我们画连线
  if (pstCo->GmCom().WatchMeCount() > 0) {
    std::vector<VECTOR3F> ptlist;
    if (pstMsg->data().pathpointlist_size() == 1) {
      ptlist.push_back(pstCo->GetPos());
    }
    for (int i = 0; i < pstMsg->data().pathpointlist_size(); ++i) {
      ptlist.push_back(v3f::FromPbv3(pstMsg->data().pathpointlist(i)));
    }

    pstCo->GmCom().ForEachWatchMeRole([&](ACombatRole &role, int mode) {
      if (mode == WATCH_MODE_ALL || mode == WATCH_MODE_MOVE) {
        role.GmCom().DrawLineList(ptlist, g_view_path_time, pxhelp::kBlueColor);
        // 画路径朝向箭头
        for (int i = 0; i < data_.path_point_list_count; ++i) {
          //
          VECTOR3F start = data_.path_point_list[i].pos;
          VECTOR3F end = start + data_.path_point_list[i].dir * 50;
          role.GmCom().DrawDirArrow(start, end, 500, g_view_path_time);
        }
      }
      return true;
    });
  }

  if (include_self) {
    VisionBroadcastCSSubMsg(pstCo, stPkg, V_FILTER_VISION_INCLUDE_SELF);
  } else {
    VisionBroadcastCSSubMsg(pstCo, stPkg, V_FILTER_VISION_EXCLUDE_SELF);
  }

  LogTraceM(
      LOGM_MOVE,
      "BROADCAST MOVE status(%s) reason(%s) map_id(%d) moveSpeed:%ju, speedattr(%ju) entity(%d), "
      "dir(%s), nav reason(%d), timestamp(%s), curpos(%s), rotating(%d)",
      MoveStateStr(data_.cur_state), BroadcastReason_Name(reason).c_str(), GetMapResID(),
      GetCoOwner()->GetMoveSpeed(), GetCoOwner()->GetAttr(ATTR_MOVESPEED), GetEntityType(),
      V3FSTR(GetCoOwner()->PosCom().GetDir()), GetCoOwner()->NavCom().GetNavReason(),
      DateTimeStr(pstMsg->timestamp() / 1000), V3FSTR(GetPos()), IsRotating());
  // LogTraceM(LOGM_MOVE, "%s", stPkg.DebugString().c_str());

  return 0;
}

void CMoveComponent::BroadcastSpeedChange() {
  float time;
  GenCostTime(nullptr, &time);
  BroadcastMsg(BroadcastReason_SpeedChange);
}

void CMoveComponent::OnTickRotate(uint64_t &left_ms) {
  if (!IsRotating()) {
    return;
  }

  auto curdir = GetCoOwner()->GetWorldDir();
  auto desdir = GetCoOwner()->LocalDirToWorld(data_.target_dir);
  int total_degree = v3f::GetDegree(curdir, desdir, true);
  const auto *modelcfg = GetCoOwner()->GetModelCfg();
  // 如果配置时间为空，则不需要转身时间
  if (!modelcfg || modelcfg->rotatespeed() <= 0.01f) {
    return;
  }
  // 减去最小转身角度
  total_degree = (total_degree - modelcfg->minrotatedegree()) * (total_degree < 0 ? -1 : 1);
  int change_degree = modelcfg->rotatespeed() * left_ms / 1000.0f;

  std::string tip;
  if (CheckLogSwitch(LOG_DEBUG)) {
    tip = format_string(
        "obj rotate, objid:%lu, worlddir:%s-->%s, change_degree:(%d= spd:%f * ms:%ld), "
        "total_degree:%d",
        GetOwnerid(), V3FSTR(curdir), V3FSTR(desdir), change_degree, modelcfg->rotatespeed(),
        left_ms, total_degree);
  }

  if (change_degree >= abs(total_degree)) {
    // 转向全部完成，时间可能有剩余
    GetCoOwner()->PosCom().SetDir(GetTarDir(), SetDirReason_StepRotate, false);

    // 计算剩余时间
    left_ms = (change_degree - abs(total_degree)) / modelcfg->rotatespeed() * 1000.0f;
    data_.last_rotate_time = 0;
    data_.rotate_src_skill_id = 0;
    LogTraceM(LOGM_MOVE, "obj:%lu rotate finish, %s, left_ms:%ld", GetOwnerid(), tip.c_str(),
              left_ms);
  } else {
    // 转向未完成，下次tick还要继续转，时间消耗完毕
    // 注意，这里的sign用于ue的旋转, ue:y+=朝下，degree：y+=朝上
    // 如朝左下（-0.5,-1.0,0)--》deg=116为正数，代表顺时针旋转，
    // 而对应到ue中，朝下即朝着y+方向，即为逆时针，应转为负数
    int sign = total_degree > 0 ? -1 : 1;
    VECTOR3F interdir_world =
        pxhelp::GetPosByRotate(VECTOR3F(0, 0, 0), curdir, change_degree * sign, 1);
    VECTOR3F interdir_local = GetCoOwner()->WorldDirToLocal(interdir_world);
    GetCoOwner()->PosCom().SetDir(interdir_local, SetDirReason_StepRotate, false);
    left_ms = 0;
    LogTraceM(LOGM_MOVE, "obj:%lu rotate steping, %s, interdir:(l:%s,w:%s), left_ms:%lu",
              GetOwnerid(), tip.c_str(), V3FSTR(interdir_local), V3FSTR(interdir_world), left_ms);
  }
}

void CMoveComponent::DoOnPath(uint64_t left_ms, float speed, OUT VECTOR3F &pos) {
  left_ms = MIN(left_ms, 3000);  // 每次不超过3s
  pos = GetPos();
  // 旋转时间h
  OnTickRotate(left_ms);
  if (left_ms == 0) {
    return;
  }

  // 走到这里，说明这个tick中已经旋转完毕，需要继续位移操作
  float fPassDist = left_ms * speed / 1000.0;

  int iDelIndex = -1;
  for (int i = 0; i < GetPathPointCount(); ++i) {
    VECTOR3F stTar = data_.path_point_list[i].pos;
    float fToNextDist = v3f::Dist(pos, stTar);

    if (fToNextDist <= fPassDist) {
      // 到达或超过这个点
      pos = stTar;
      fPassDist -= fToNextDist;
      iDelIndex = i;
      LogTraceM(LOGM_MOVE,
                "obj:%lu, move arrive end, index:%d, cur path point(%s), curdist:%f newpos(%s)",
                GetOwnerid(), i, v3f::get_str(stTar).c_str(), fToNextDist,
                v3f::get_str(pos).c_str());
    } else {
      pos = v3f::GetEndByEndpt(pos, stTar, fPassDist);
      LogTraceM(LOGM_MOVE, "obj:%lu cur path point(%s), curdist:%f, newpos(%s)", GetOwnerid(),
                v3f::get_str(stTar).c_str(), fPassDist, v3f::get_str(pos).c_str());
      break;
    }
  }

  // 走过的点直接删除
  if (iDelIndex >= 0 && GetPathPointCount() >= (iDelIndex + 1)) {
    int iMoveUnitCount = GetPathPointCount() - (iDelIndex + 1);
    memmove(data_.path_point_list, &data_.path_point_list[iDelIndex + 1],
            iMoveUnitCount * sizeof(data_.path_point_list[0]));
    data_.path_point_list_count = data_.path_point_list_count - (iDelIndex + 1);
    if (GetPathPointCount() <= 1) {
      LogTraceM(LOGM_MOVE, "del move point objid:%ld, leftCount:%d,isfinish: %d ", GetOwnerid(),
                GetPathPointCount(), (GetPathPointCount() == 0 ? 1 : 0));
    }
  }

  // 备份当前寻路点
  auto bp_ai_com = GetOwner()->GetCom<CMonBpAICom>();
  if (bp_ai_com && bp_ai_com->Info().is_record) {
    bp_ai_com->Info().path_point_list_count = 0;
    for (int i = 0; i < data_.path_point_list_count; ++i) {
      bp_ai_com->Info().add_path_point_list(data_.path_point_list[i]);
    }
  }
}

#define MIN_ROTATE_DEGREE 2.0f
bool CMoveComponent::IsRotating() const {
  // 如果是DetourCrowd算法控制的对象直接忽略，因为算法计算的时候已经考虑到了
  if (GetCoOwner()->NavCom().IsCrowdAlgoControlAgent()) {
    return false;
  }
  // 靶向移动的时候，不原地转身
  if (GetCoOwner()->MoveCom().IsTargetMove()) {
    return false;
  }
  if (GetCoOwner()->NavCom().GetNavReason() == NAVI_REASON_BT_PURSUIT ||
      GetCoOwner()->NavCom().GetNavReason() == NAVI_REASON_SKILL_PURSUIT) {
    return false;
  }
  float degree = v3f::GetDegree(GetCoOwner()->PosCom().GetDir2D(), GetTarDir2D(), false);
  if (degree < MIN_ROTATE_DEGREE) {
    return false;
  }
  const auto *modelcfg = GetCoOwner()->GetModelCfg();
  if (modelcfg && modelcfg->minrotatedegree() >= 0.01f &&
      (degree - modelcfg->minrotatedegree()) < MIN_ROTATE_DEGREE) {
    return false;
  }
  return true;
}

bool CMoveComponent::IsMoving() const {
  if (GetCoOwner()->NavCom().IsCrowdAlgoControlAgent()) {
    return !GetCoOwner()->NavCom().IsNearDestPoint();
  }
  if (IsRotating()) {
    return true;
  }
  return GetPathPointCount() > 0;
}

void CMoveComponent::ClearPathList() {
  // 清理寻路路径
  data_.path_point_list_count = 0;
}

int CMoveComponent::TickDelay() {
  auto &delay = data_.delay_rotate;

  // no action
  if (delay.trigger_time <= 0) {
    return 0;
  }

  // time not arrive
  if (GetRealTimeMs() < delay.trigger_time) {
    return 0;
  }

  // arrive, if not moving do it
  if (!IsMoving()) {
    LogDebugM(LOGM_MOVE, "start delay rotate");
    RotateStep(delay.target_dir, static_cast<NAVI_REASON>(delay.reason), delay.min_time,
               delay.speed, delay.src_skill_instance_id);
  }
  delay.Clear();
  return 0;
}

int CMoveComponent::OnPath(int time_in_advance) {
  ACombatActor *pstCo = GetCoOwner();
  if (pstCo == nullptr) {
    return -1;
  }

  // if (pstCo->IsRole()) {
  //   return -1;
  // }
  if (!pstCo->MoveCom().IsServerPathfinding() /*|| pstCo->IsDead()*/) {
    return -1;
  }
  if (pstCo->IsDisableMove()) {
    return 0;
  }

  // 走完了 TODO(walterwwang)整理为走状态
  if (!IsMoving()) {
    return OnFinishPath();
  }

  // 禁止移动，速度为0, speed值的单位是cm
  float speed = GetCoOwner()->GetMoveSpeed();
  if (speed <= 0) {
    LogTraceM(LOGM_MOVE, "co:%ld, speed:%f", GetOwnerid(), speed);
    ClearPathList();
    return -1;
  }

  // 过去时间太短
  auto &poscom = GetCoOwner()->PosCom();
  int64_t ullDiffMs = GetRealTickTimeMs() - poscom.GetLastMoveTime() + time_in_advance;
  if (ullDiffMs < 10) {
    return -1;
  }
  //  LogDebugM(LOGM_MOVE, "co:%ld, on path ing", GetOwnerid());

  VECTOR3F finalPos = GetPos();
  DoOnPath(ullDiffMs, speed, finalPos);

  bool changedir = true;
  // 靶向移动以靶向方向为准
  if (GetCoOwner()->MoveCom().IsTargetMove()) {
    auto target_id = GetCoOwner()->GetAttr(ATTR_TARGET_OBJ_ID);
    auto *target = GetObjectFactory().FindOrigCo(target_id);
    if (target) {
      const auto &target_pos = target->GetPos();
      const auto &dir = target_pos - GetCoOwner()->GetPos();
      poscom.SetDir(v3f::Normalized(dir), SetDirReason_UpdatePos);
      changedir = false;
    }
  }

  poscom.SetLastMoveTime(GetRealTickTimeMs());
  if (finalPos != GetPos()) {
    poscom.UpdatePos(finalPos, changedir, kUpdatePosReason_MoveOnPath);
  }
  /*if (GetCoOwner()->GetResid() == 21040244 || GetCoOwner()->GetResid() == 21000074) {
    LogErrorM(LOGM_MOVE, "co:%ld, map_id(%d) curpos(%s) time:%ld, speed:%f", GetOwnerid(),
              GetMapResID(), V3FSTR(GetPos()), ullDiffMs, speed);
  }*/

  return 0;
}

void CMoveComponent::FillCsMoveComInfo(CSMoveInfo &out, bool is_moving) {
  out.set_curstate(data_.cur_state);
  out.set_rotateinplace(IsRotating());
  out.set_reason(data_.reason);
  int start_index = 0;
  // 非角色的战斗体在路径点大于2的时候，第一个点往往是当前点，这里把当前点去掉，避免延迟造成客户端往回走
  // 2022-9-1 在怪物上次移动路径走完后，再次寻路的时候，不删除第一个点，同步所有的点
  /*if (GetCoOwner()->MoveCom().IsServerPathfinding() && GetPathPointCount() >= 2 && is_moving) {
    start_index = 1;
    if (GetOwner()->IsRole()) {
      start_index = 0;
    }
  }*/
  ACombatActor *pActor = GetCoOwner();
  for (int i = start_index; i < GetPathPointCount(); ++i) {
    data_.path_point_list[i].pos.CopyToPb(*out.add_pathpointlist());
    VECTOR3F dir = data_.path_point_list[i].dir;
    pActor->PosCom().FixDirNoUpward(dir);
    dir.CopyToPb(*out.add_dirlist());
  }
  VECTOR3F target_dir = data_.target_dir;
  pActor->PosCom().FixDirNoUpward(target_dir);
  target_dir.CopyToPb(*out.mutable_targetdir());
  if (pActor->MoveCom().IsTargetMove()) {
    if (out.dirlist_size() > 1) {
      *out.mutable_dir() = out.dirlist(0);
    }
  }
  out.set_targetobjid(pActor->MoveCom().GetTargetObjID());
  // 待客户端改造后删除 zzhijie 20230425
  out.set_istargetmove(IsTargetMove());
}

void CMoveComponent::SyncMapInfo(bool is_transfer) {
  CSMsgMove moveinfo;
  GetCoOwner()->FillCsMoveInfo(*moveinfo.mutable_fullinfo());
  moveinfo.set_timestamp(GetCoOwner()->PosCom().GetLastMoveTime());
  moveinfo.mutable_fullinfo()->set_istransfer(is_transfer);

  int sky_color = 0;
  auto ptr_map = GetCoOwner()->GetMapinst();
  if (ptr_map) {
    sky_color = ptr_map->GetSkyColorID();
  }
  moveinfo.mutable_fullinfo()->set_skycolorid(sky_color);
  LogDebugM(LOGM_MOVE, "move sky_id:%d info: %s", sky_color, moveinfo.DebugString().c_str());
  SendCSSubMsg(GetCoOwner(), moveinfo);
}

void CMoveComponent::OnTick1sec() {
  auto *owner = GetCoOwner();
  if (!owner->IsRole()) {
    return;
  }

  // 记录tick时间
  auto curr_time = GetRealTickTimeMs();
  auto last_time = data_.last_energy_recover_tick;
  auto time_diff = curr_time - last_time;
  data_.last_energy_recover_tick = curr_time;

  // 托管到坐骑组件的让坐骑来处理体力
  auto *mount_com = owner->GetCom<CMountComponent>();
  if (mount_com && mount_com->IsOnMount()) {
    return;
  }

  // 体力恢复逻辑
  int cur_energy = owner->GetAttr(ATTR_CURENERGY);
  int max_energy = owner->GetAttr(ATTR_MAXENERGY);
  bool need_recover = false;
  auto cur_state = GetCurState();
  auto sub_state = GetSubState();
  switch (cur_state) {
    case MOVE_TYPE_SWIM: {
      if (sub_state == MoveSubType_Normal) {         // 正常状态才恢复体力
        need_recover = true;
      } else if (sub_state == MoveSubType_Sprint) {  // 快速游泳扣除体力
        int cost = GetGlobalCfgValue(RES_GLOBAL_ROLE_SPRINT_ENERGY_COST);
        ConsumeEnergy(cost);
      }
      break;
    }
    case MOVE_TYPE_WALK:
    case MOVE_TYPE_STOP: {
      need_recover = true;
      break;
    }
    default:
      break;
  }

  // 消耗体力指定周期内不回复体力
  if (curr_time - data_.last_energy_consume_time <= ENERGY_RECOVE_GAP_MS) {
    return;
  }

  // 上个周期未恢复体力
  if (last_time - data_.last_energy_consume_time <=
      ENERGY_RECOVE_GAP_MS) {  // 上个周期还没开始恢复体力
    time_diff =
        curr_time - (data_.last_energy_consume_time + ENERGY_RECOVE_GAP_MS);  // 计入恢复的世界
  }

  // 恢复体力
  if (need_recover) {
    int energy_per_sec = owner->GetAttr(ATTR_ENERGY_RECOVER);
    cur_energy = MIN(max_energy, cur_energy + (energy_per_sec * time_diff / 1000.0f));
    owner->SetAttr(ATTR_CURENERGY, cur_energy);
  }
}

void CMoveComponent::ModifyPath(vector<VECTOR3F> &path, VECTOR3F *ref_point) {
  auto monster = GetCoOwner()->ToMonster();
  if (nullptr == monster) {
    return;
  }
  const auto *modelcfg = GetCoOwner()->GetModelCfg();
  if (modelcfg == nullptr) {
    LogTraceM(LOGM_MOVE, "monster (%lu) has no model cfg, not smoothing",
              GetCoOwner()->GetAttr(ATTR_RESID));
    return;
  }
  vector<VECTOR3F> output;
  bool succ = GetNavSvc().SmoothPath(path, output, modelcfg->interpolatesegment(),
                                     modelcfg->interpolatefactor());

  if (succ) {
    LogTraceM(LOGM_MOVE, "smooth path count(%lu), path count before(%lu)", output.size(),
              path.size());
    LogTraceM(LOGM_MOVE, "combined path count(%lu), path count before(%lu)", output.size(),
              path.size());
    ClearPathList();

    // GetNavSvc().CombinePath(output);
    path.swap(output);
  }
}

void PrintPos(AVisionActor &obj, vector<VECTOR3F> &path, const char *tip) {
  std::string log;
  for (auto pos : path) {
    AppendString(log, "%s, ", V3FSTR(pos));
  }

  LogInform("%ld, cur:%s, path:%s, tip:%s", obj.GetID(), V3FSTR(obj.GetPos()), log.c_str(), tip);
}

// 实体公共状态变化监听
void CMoveComponent::OnEntityCommstatChanged(const EVENTCONTEXT &evt) {
  if (GetOwnerType() != GID_ROLE) {
    return;
  }
  const auto &commstat = evt.event.entity_commstat_changed;
  if (IsSneaking()) {  // 潜行中，判断是否有不可潜行的新状态
    for (int i = 0; i < commstat.enter_statuses_count; ++i) {
      auto status = commstat.enter_statuses[i];
      if ((status == RES_COMMSTAT_BATTLE) || (status == RES_COMMSTAT_GATHER) ||
          (status == RES_COMMSTAT_DEAD) || (status == RES_COMMSTAT_NEAR_DEAD)) {
        StopSneaking();
        break;
      }
    }
  } else {  // 尝试进入潜行
    for (int i = 0; i < commstat.exit_statuses_count; ++i) {
      auto status = commstat.exit_statuses[i];
      if ((status == RES_COMMSTAT_BATTLE) || (status == RES_COMMSTAT_GATHER) ||
          (status == RES_COMMSTAT_DEAD) || (status == RES_COMMSTAT_NEAR_DEAD)) {
        TryStartSneaking();
        break;
      }
    }
  }
}

// 视野变化监听
void CMoveComponent::OnVisionUpdate(const EVENTCONTEXT &evt) {
  if (GetOwnerType() != GID_ROLE) {
    return;
  }
  const auto &vision_upadte = evt.event.vision_upadte;
  auto obj_type = GID_TO_GID_TYPE(vision_upadte.obj_id);
  if (obj_type != GID_MONSTER) {  // 怪物需要判断是否进入潜行
    return;
  }
  // 加入视野
  if (vision_upadte.add_or_del) {
    TryStartSneaking();
  } else {  // 退出视野
    if (IsSneaking()) {
      // 视野内无野怪则退出潜行
      VISIONFOREACHPARAM param;
      GetCoOwner()->VisionCom().GenVisionForEachParam(param);
      if (!VisionHasObjType(param, V_FOREACH_MONSTER)) {
        StopSneaking();
      }
    }
  }
}

// 是否满足潜行条件
bool CMoveComponent::CanStartSneaking() {
  if (GetOwnerType() != GID_ROLE) {
    return false;
  }
  ACombatRole *role = GetRole();
  if (role == nullptr) {
    return -1;
  }
  if (GetCoOwner()->IsGhost()) {
    return false;
  }

  // 公共状态过滤
  const auto &commstat_com = role->CommstatCom();
  if (commstat_com.Is(RES_COMMSTAT_BATTLE) || commstat_com.Is(RES_COMMSTAT_GATHER) ||
      commstat_com.Is(RES_COMMSTAT_DEAD) || commstat_com.Is(RES_COMMSTAT_NEAR_DEAD)) {
    return false;
  }

  // 玩家移动状态过滤
  auto cur_state = GetCurState();
  auto sub_state = GetSubState();
  switch (cur_state) {
    case MOVE_TYPE_STOP: {  // 停止并在地面才能进入
      if (GetCoOwner()->GetAttr(ATTR_MOVE_VERTICAL_LOCATION) == MOVE_VERTICAL_LOCATION_GROUND) {
        return false;
      }
      break;
    }
    case MOVE_TYPE_WALK: {  // 普通步行可以进入
      if (sub_state != MoveSubType_Sprint) {
        return false;
      }
      break;
    }
    default:
      break;
  }

  // 仇恨列表是双向的，只需要看自己身上仇恨列表是否为空
  auto &taunt_com = GetCoOwner()->TauntCom();
  if (!taunt_com.IsEmpty()) {
    return false;
  }

  // 不在草地内
  if (!IsInGrass()) {
    return false;
  }

  // 视野内没有野怪，不需进入
  VISIONFOREACHPARAM param;
  GetCoOwner()->VisionCom().GenVisionForEachParam(param);
  if (!VisionHasObjType(param, V_FOREACH_MONSTER)) {
    return false;
  }

  return true;
}
// 尝试进入潜行状态
bool CMoveComponent::TryStartSneaking() {
  if (GetOwnerType() != GID_ROLE)
    return false;
  if (IsSneaking()) {
    return false;
  }
  if (!CanStartSneaking()) {
    return false;
  }

  LogDebug("StartSneaking, %ld", GetOwnerid());
  // 属性设置
  GetCoOwner()->SetAttr(ATTR_DYN_SNEAK, 1);
  // 增加隐身buff
  auto buff_id = GetGlobalCfgValue(RES_GLOBAL_SNEAK_BUFF_ID);
  AddBuff(*GetCoOwner(), buff_id, 1, -1);
  return true;
}
// 退出潜行状态
void CMoveComponent::StopSneaking() {
  if (IsSneaking()) {
    LogDebug("StopSneaking, %ld", GetOwnerid());
    // 清空属性
    GetCoOwner()->SetAttr(ATTR_DYN_SNEAK, 0);
    // 清空buff
    auto buff_id = GetGlobalCfgValue(RES_GLOBAL_SNEAK_BUFF_ID);
    RemoveBuff(*GetCoOwner(), buff_id);
  }
}

// 是否潜行中
bool CMoveComponent::IsSneaking() { return (GetCoOwner()->GetAttr(ATTR_DYN_SNEAK) != 0); }

// 是否在草地内
bool CMoveComponent::IsInGrass() { return data_.is_in_grass; }

void CMoveComponent::RefillInnerPointForHeight(VECTOR3F a, VECTOR3F b, std::vector<VECTOR3F> &out) {
  const float kFixMinPathPointDist = 210;  // 小于这个距离，就不新增点了
  const float kStepDist = 150;             // a朝b方向，每次递进的3d距离
  // 太近不调整
  if (v3f::IsDistInRange(a, b, kFixMinPathPointDist)) {
    return;
  }

  auto *pxcom = GetOwner()->GetCom<PhysXComponent>();
  ERR_IF_NULL_SIMP(pxcom, (void()));

  auto step_dir = v3f::Normalized(b - a) * kStepDist;
  auto max_dist = v3f::Dist(a, b);
  if (max_dist > 10000) {
    return;
  }
  int step_index = 0;
  VECTOR3F curpos = a;
  int looptime = 0;
  while (true) {
    DEAD_LOOP_TIMES_BREAK(looptime, 50);
    ++step_index;
    if (step_index * kStepDist >= max_dist) {
      break;
    }

    curpos = curpos + step_dir;

    VECTOR3F groundpos = curpos;
    if (!pxcom->FixPointHeightByClientTolerance(groundpos)) {
      continue;
    }

    out.push_back(groundpos);
    auto func = [&](ACombatRole &role, int mode) {
      role.GmCom().DrawDirArrow(curpos, groundpos, 500, g_view_path_time, pxhelp::kYellowColor);
      return true;
    };
    GetCoOwner()->GmCom().ForEachWatchMeRole(func);
  }

  if (!out.empty()) {
    std::string poslist;
    for (auto p : out) {
      AppendString(poslist, "(%s)", V3FSTR(p));
    }
    LogTrace("obj:%lu,%s, add inter point, original:%s->%s, add:%ld, list:%s", GetOwnerid(),
             GetOwner()->GetObjName(), V3FSTR(a), V3FSTR(b), out.size(), poslist.c_str());
  }
}

void CMoveComponent::FixPathPointListHeight(vector<VECTOR3F> &path) {
  vector<VECTOR3F> oldlist;
  vector<VECTOR3F> newpath;
  oldlist.swap(path);
  auto *pxcom = GetOwner()->GetCom<PhysXComponent>();
  ERR_IF_NULL_SIMP(pxcom, (void()));

  VECTOR3F lastpos = GetPos();
  for (auto &curpos : oldlist) {
    // 补充路径上点
    std::vector<VECTOR3F> addpos;
    RefillInnerPointForHeight(lastpos, curpos, addpos);
    if (!addpos.empty()) {
      newpath.insert(newpath.end(), addpos.begin(), addpos.end());
    }

    // 当前点
    pxcom->FixPointHeightByClientTolerance(curpos);
    newpath.push_back(curpos);

    lastpos = curpos;
  }

  path.swap(newpath);
}

int CMoveComponent::SetNavmeshPath(vector<VECTOR3F> &path, int coord_type, int duration,
                                   VECTOR3F *ref_point, VECTOR3F *face_point,
                                   MOVE_TYPE move_state) {
  ClearPathList();
  if (path.empty()) {
    LogTraceM(LOGM_MOVE, "path size <= 0");
    return -1;
  }

  // PrintPos(*GetVisiOwner(), path, "before SmoothPath");
  bool is_moving = IsMoving();
  // 1.路径平滑
  // 这里路径平滑的时候，新增的点并没有考虑是否落到navmesh网格的片面上，后续需要优化下TODO(henrycao)
  ModifyPath(path, ref_point);
  // 如果使用了下一个点来辅助平滑，那先把后面那段辅助的点删除
  if (ref_point) {
    for (auto it = path.begin(); it != path.end(); ++it) {
      if (v3f::IsDistInRange(*ref_point, *it, 100.0f)) {
        path.erase(++it, path.end());
        break;
      }
    }
  }

  // 临时代码，ab2点之间有点距离地面过高，超过一定距离，新增贴地点，保证不会浮空
  // PrintPos(*GetVisiOwner(), path, "before fix too high");
  FixPathPointListHeight(path);
  // PrintPos(*GetVisiOwner(), path, "after fix too high");

  // PrintPos(*GetVisiOwner(), path, "after fix too high");
  //  2.先把路径从navmesh里取出，此时是世界(Root)坐标
  for (int i = 0; i < static_cast<int>(path.size()) && i < COUNTOF(data_.path_point_list); ++i) {
    auto &point = path[i];
    data_.path_point_list[i].pos = {point.x, point.y,
                                    point.z + GetCoOwner()->GetCenterGroundDist()};
    ++data_.path_point_list_count;
  }

  // 引路等行为中需要备份寻路点信息
  auto bp_ai_com = GetOwner()->GetCom<CMonBpAICom>();
  if (bp_ai_com && bp_ai_com->Info().is_record) {
    bp_ai_com->Info().path_point_list_count = 0;
    for (int i = 0; i < data_.path_point_list_count; ++i) {
      bp_ai_com->Info().add_path_point_list(data_.path_point_list[i]);
    }
  }

  SetTarDirData(data_.path_point_list[0].pos - GetPos(), SetDirReason_AfterNavi);

  //  PrintPos(*GetVisiOwner(), path, "3333");
  // 2.计算下路径长度和时间，包括原地旋转时间
  float fDistance = 0;
  float total_time = 0;
  float rotate_ms = 0;
  GenCostTime(&fDistance, &total_time, 0, 0, &rotate_ms);
  if (IS_FLT_ZERO(total_time)) {
    return 0;
  }

  if (coord_type == PFMeshType_Vehicle) {
    ERR_IF_NULL_M(LOGM_NAVI, GetCoOwner()->GetVehicle(), -1,
                  "obj is expected on a vehicle, but vehicle(%ju) is not found",
                  GetCoOwner()->GetVehicleGid());
  }

  // 填充朝向数组
  FillPointDir(face_point);

  // 开始移动
  SetCurState(move_state);

  // 寻路终点碰撞检查, 修正终点坐标
  auto des_pos = GetDestPoint();
  auto fix_dis = GetDestPoint(1) - des_pos;
  auto ret = GetCoOwner()->PhysXCom().PathFindingOverlapFix(des_pos, fix_dis);
  if (ret && GetPathPointCount() > 0) {
    data_.path_point_list[GetPathPointCount() - 1].pos = des_pos;
    LogDebugM(LOGM_MOVE, "toucher suc! pos[%s] -> [%s]", V3FSTR(GetDestPoint()), V3FSTR(des_pos));
  }

  // 更新 physx actor pos
  UpdateActorPosMark();

  // log
  // std::string futuretime = DateTimeStr(GetFutureStopTime() / 1000);
  // LogDebugM(
  //     LOGM_MOVE,
  //     "navmesh path set, speed(%ld) dist(%.5f), destime(%s.%ld), nowtime(%s.%ld),delta(%f),first
  //     " "point (%s), end point(%s). point count(%d), init dir(%s), cur dir(%s), reason(%d) "
  //     "des_pos(%s)",
  //     GetCoOwner()->GetMoveSpeed(), fDistance, futuretime.c_str(), GetFutureStopTime() % 1000,
  //     DateTimeStr(GetTickTimeS()), GetRealTickTimeMs() % 1000, total_time,
  //     V3FSTR(data_.path_point_list[0].pos),
  //     V3FSTR(data_.path_point_list[data_.path_point_list_count - 1].pos),
  //     data_.path_point_list_count, V3FSTR(v3f::Normalized(data_.path_point_list[0].pos -
  //     GetPos())), V3FSTR(GetCoOwner()->PosCom().GetDir()), GetCoOwner()->NavCom().GetNavReason(),
  //     V3FSTR(des_pos));
  BroadcastMsg(BroadcastReason_NavmeshFin, false, is_moving);

  return 0;
}

int CMoveComponent::SetSimplePath(VECTOR3F start, VECTOR3F end,
                                  int move_state /* = MOVE_TYPE_WALK*/) {
  ClearPathList();

  const auto *modelcfg = GetCoOwner()->GetModelCfg();
  ERR_IF_NULL_M(LOGM_NAVI, modelcfg, -1, "modelcfg is null");

  std::vector<VECTOR3F> path;
  path.push_back(start);
  if ((move_state == MOVE_TYPE_WALK || move_state == MOVE_TYPE_FLY) &&
      !IS_FLT_ZERO(modelcfg->interpolatesegment())) {
    VECTOR3F ref_point = start + GetCoOwner()->GetDir() * modelcfg->interpolatesegment();
    path.push_back(ref_point);
    path.push_back(end);

    // 路径平滑
    ModifyPath(path, &ref_point);
  } else {  // 没有配置平滑参数
    path.push_back(end);
  }

  for (auto &p : path) {
    data_.add_path_point_list({p, {0, 0, 0}});
  }

  ERR_IF_FALSE_M(LOGM_NAVI, data_.path_point_list_count > 1, -1, "path count(%d) is invalid",
                 data_.path_point_list_count);

  // 设置目标朝向
  SetTarDirData(data_.path_point_list[1].pos - data_.path_point_list[0].pos,
                SetDirReason_AfterNavi);
  GetOwner()->SetAttr(ATTR_FINDPATH_FAIL_TIMES, 0);

  // 填充朝向数组
  FillPointDir(nullptr);

  // 开始移动
  SetCurState(move_state);

  // 2.计算下路径长度和时间，包括原地旋转时间
  float fDistance = 0;
  float fTime = 0;
  GenCostTime(&fDistance, &fTime);
  if (IS_FLT_ZERO(fTime)) {
    return 0;
  }

  if (move_state != MOVE_TYPE_STOP) {
    GetCoOwner()->PosCom().SetLastMoveTime(GetRealTickTimeMs());
  }

  // log
  std::string futuretime = DateTimeStr(GetFutureStopTime() / 1000);
  LogTraceM(LOGM_MOVE,
            "navmesh path set, objid(%ju), speed(%ld) dist(%.5f), "
            "destime(%s), nowtime(%s),delta(%f),first point (%s), end point(%s) init dir(%s), cur "
            "dir(%s), reason(%d)",
            GetOwnerid(), GetCoOwner()->GetMoveSpeed(), fDistance, futuretime.c_str(),
            DateTimeStr(GetTickTimeS()), fTime, V3FSTR(data_.path_point_list[0].pos),
            V3FSTR(data_.path_point_list[GetPathPointCount() - 1].pos),
            V3FSTR(v3f::Normalized(data_.path_point_list[0].pos - GetPos())),
            V3FSTR(GetCoOwner()->PosCom().GetDir()), GetCoOwner()->NavCom().GetNavReason());
  BroadcastMsg(BroadcastReason_NavmeshFin, true);

  return 0;
}

float CMoveComponent::GetRotateTimeMs(int &degree, float min_time, float speed) const {
  // debug 如果在战兽上，不转身, detour crowd算法控制的agent也不转身
  if (GetCoOwner()->IsOnVehicle() || GetCoOwner()->NavCom().IsCrowdAlgoControlAgent()) {
    return 0;
  }

  auto &poscom = GetCoOwner()->PosCom();

  auto curdir = poscom.GetDir();
  auto desdir = GetTarDir2D();
  degree = v3f::GetDegree(curdir, desdir, false);

  auto reason = GetCoOwner()->NavCom().GetNavReason();
  if (reason == NAVI_REASON_ROTATE_SKILL) {
    // skill rotate, time by speed
    float costms = degree / MAX(1, speed) * 1000;
    return MAX(costms, min_time * 1000);
  }

  // 如果配置时间为空，则不需要转身时间
  const auto *modelcfg = GetCoOwner()->GetModelCfg();
  ERR_IF_NULL_SIMP(modelcfg, 0);
  if (modelcfg->rotatespeed() <= 0.01f) {
    return 0;
  }

  float rotatems = 0;
  // rotate speed 单位角度/s
  rotatems = degree / modelcfg->rotatespeed() * 1000.0f;
  // 有转身时间
  if (reason == NAVI_REASON_BT_PURSUIT) {
    // TODO(walterwang) use globalcfg
    if (degree < 60) {
      // 小角度追击
      return 0;
    }

    // 大角度追击
    if (GetCoOwner()->MoveCom().IsServerPathfinding()) {
      // TODO(xibatan) 后续使用modelCfg判定“追击时是否需要转身”
      // auto res = GetCoOwner()->GetModelCfg();
      // if (res && !res->is_rotating_when_pursuit) {
      //   return 0;
      // }
      if (!GetCoOwner()->IsMonster()) {
        return 0;
      }
      const RESMONSTERDEF *res = GetCoOwner()->ToMonster()->FindRes();
      if (res && !res->is_rotating_when_pursuit) {
        return 0;
      }
    }
  } else if (reason == hx2proto::NAVI_REASON_BT_FOLLOW) {
    // 如果是跟随，就不进行转身了，防止在转身中需要时间，导致跟随卡顿
    return 0;
  }

  // 其他情况，有配置则默认转身
  return rotatems;
}

void CMoveComponent::GenCostTime(float *dist /*= nullptr*/, float *time /*= nullptr*/,
                                 float rotate_min_time /*= 0*/, float rotate_speed /*= 0*/,
                                 float *output_rotate_time /*= 0*/) {
  float fDist = 0;
  float cost_time_ms = 0;

  // 1.计算行走路径的时间
  // 如果当前路径的第一个点离当前点很近，会将路径的第一个点去掉，此时路径点列表中仅剩一个点，这里计算
  // 距离的话就是0，就不会给客户端广播移动消息，导致表现异常；所以这里需要将实体当前位置与路径的第一
  // 点的距离也加进来
  if (data_.path_point_list_count > 0) {
    fDist += v3f::Dist(GetPos(), data_.path_point_list[0].pos);
  }
  for (int i = 1; i < GetPathPointCount() && i < COUNTOF(data_.path_point_list); ++i) {
    fDist += v3f::Dist(data_.path_point_list[i - 1].pos, data_.path_point_list[i].pos);
  }
  float speed = MAX(GetCoOwner()->GetMoveSpeed(), 1);
  cost_time_ms = fDist / speed * 1000;

  // 2.计算寻路首次转向的时间
  int degree = 0;
  auto rotatems = GetRotateTimeMs(degree, rotate_min_time, rotate_speed);
  LogTraceM(
      LOGM_MOVE,
      "dist(%.2f) speed(%.2f), path time(%.2f), degree(%d), rotate time(%.2f) total time(%.2f)",
      fDist, speed, cost_time_ms, degree, rotatems, cost_time_ms + rotatems);
  cost_time_ms += rotatems;

  // ok---
  if (dist) {
    *dist = fDist;
  }
  if (time) {
    *time = cost_time_ms;
  }
  if (cost_time_ms <= 0.01f) {
    return;
  }
  if (output_rotate_time) {
    *output_rotate_time = rotatems;
  }
  int64_t cur_time = GetRealTickTimeMs();
  data_.future_stop_time = cur_time + static_cast<uint64_t>(cost_time_ms);
  // LogTraceM(LOGM_MOVE, "cur time(%s)", DateTimeStr(cur_time / 1000));
  LogTraceM(LOGM_MOVE, "path end time(%s)", DateTimeStr(GetFutureStopTime() / 1000));
  // 转向数据
  data_.last_rotate_time = GetRealTickTimeMs();
}

bool CMoveComponent::IsStatus(int commstat_type) const {
  switch (commstat_type) {
    case RES_COMMSTAT_FLY:
      return (GetCurState() == MOVE_TYPE_GLIDE || GetCurState() == MOVE_TYPE_FLY ||
              GetCurState() == MOVE_TYPE_FREE_FALL);

    case RES_COMMSTAT_MOVING:
      return GetCurState() == MOVE_TYPE_WALK;

    default:
      break;
  }

  return false;
}

int CMoveComponent::RotateImmediate(int reason, Vector3F target_dir, uint64_t target_time) {
  // 暂时处理一下客户端的请求，服务器主动Rotate在tick中实现
  VECTOR3F tardir;
  tardir.CopyFromPb(target_dir);
  SetTarDirData(tardir, SetDirReason_Rotate);
  GetCoOwner()->PosCom().SetLastMoveTime(target_time);

  SetCurState(MOVE_TYPE_CHANGE_DIR);
  BroadcastMsg(BroadcastReason_Rotate);

  SetCurState(MOVE_TYPE_STOP);

  GetCoOwner()->PosCom().SetDir(v3f::FromPbv3(target_dir), hx2proto::SetDirReason_Rotate);
  return 0;
}

int CMoveComponent::RotateStep(VECTOR3F target_dir, NAVI_REASON reason, float min_time, float speed,
                               uint32_t skillinstid) {
  target_dir = v3f::Normalized(target_dir);
  if (GetCoOwner()->PosCom().GetDir() == target_dir) {
    return 0;
  }

  // ok--
  ClearPathList();
  // 原地转向，客户端需要pathpointlist加一个当前位置，dirlist里加一个最终朝向
  if (reason == NAVI_REASON_TURN_IN_PLACE) {
    POINTNODE point;
    point.pos = GetCoOwner()->PosCom().GetPos();
    point.dir = target_dir;
    data_.add_path_point_list(point);
  }

  // 设置目标朝向
  SetCurState(MOVE_TYPE_WALK);
  SetTarDirData(target_dir);
  GetCoOwner()->NavCom().SetNavReason(reason);
  GetCoOwner()->PosCom().SetLastMoveTime(GetRealTickTimeMs());  // 重置时间，开始按照路径移动

  // 后续反馈，计算时间
  float fDistance = 0;
  float fTime = 0;
  GenCostTime(&fDistance, &fTime, min_time, speed);
  if (IS_FLT_ZERO(fTime)) {
    return 0;
  }
  data_.rotate_src_skill_id = skillinstid;

  if (reason != NAVI_REASON_ROTATE_SKILL) {
    BroadcastMsg(BroadcastReason_Rotate);
  }
  LogDebugM(LOGM_MOVE, "obj:%lu, begin rotate, tardir:%s", GetOwnerid(), V3FSTR(target_dir));
  return 0;
}

int CMoveComponent::GetEntityType() {
  if (GetCoOwner()->IsRole()) {
    return MOVE_ENTITY_ROLE;
  } else if (GetCoOwner()->IsVehicle()) {
    return MOVE_ENTITY_VEHICLE;
  } else {
    return MOVE_ENTITY_TYPE_MIN;
  }
}

// 设置靶向移动目标对象id
void CMoveComponent::SetTargetObjID(uint64_t obj_id) {
  if (obj_id == GetCoOwner()->GetAttr(ATTR_TARGET_OBJ_ID)) {
    return;
  }
  GetCoOwner()->SetAttr(ATTR_TARGET_OBJ_ID, obj_id);
  LogDebugM(LOGM_MOVE, "%lu SetTargetObjID:%lu", GetCoOwner()->GetID(), obj_id);
}

void CMoveComponent::StopRotate() {
  if (!IsRotating()) {
    return;
  }

  SetTarDirData(GetCoOwner()->PosCom().GetDir());
  data_.last_rotate_time = 0;
  data_.delay_rotate.Clear();
}

void CMoveComponent::NotifyMonsterRotateHead(uint64_t obj_id) {
  ACombatActor *obj = GetObjectFactory().FindRealCo(obj_id);
  if (nullptr == obj) {
    LogTraceM(LOGM_MOVE, "target obj(%ju) is offline", obj_id);
    return;
  }

  CSMsgMove msg;
  auto detail = msg.mutable_roteteheadchanges();
  detail->set_sourceobjid(GetOwnerid());
  detail->set_targetobjid(obj_id);
  msg.set_timestamp(GetRealTickTimeMs());

  LogTrace("uin:%lu, target:%lu", GetOwnerid(), obj_id);
  SendCSSubMsg(obj, msg);
}

// 判断一个点是否在当前路径的目标终点附近
bool CMoveComponent::IsPointNearDestPoint(VECTOR3F point, float near_dis /* = 100.0 */) {
  ACombatActor *co = const_cast<ACombatActor *>(GetCoOwner());
  VECTOR3F dest_point = data_.path_point_list[GetPathPointCount() - 1].pos;
  if (GetPathPointCount() <= 0) {
    dest_point = GetPos();
  }

  dest_point.z -= GetCoOwner()->GetCenterGroundDist();
  GetNavSvc().GetGroundPos(co, point, point);

  // 距离内
  return v3f::Dist2(dest_point, point) < near_dis * near_dis;
}
VECTOR3F CMoveComponent::GetDestPoint(int idx /*0*/) {
  if (GetPathPointCount() <= idx) {
    return GetPos();
  }
  return data_.path_point_list[GetPathPointCount() - 1 - idx].pos;
}

int CMoveComponent::DelayRotate(VECTOR3F tardir, float min_time, float speed, int64_t delay_ms,
                                int reason, uint32_t src_skillctx_instid) {
  if (delay_ms < 10) {
    return RotateStep(tardir, static_cast<NAVI_REASON>(reason), min_time, speed,
                      src_skillctx_instid);
  }
  auto &delay = data_.delay_rotate;
  delay.speed = speed;
  delay.min_time = min_time;
  delay.trigger_time = GetRealTimeMs() + delay_ms;
  delay.target_dir = tardir;
  delay.reason = reason;
  delay.src_skill_instance_id = src_skillctx_instid;
  LogDebugM(LOGM_MOVE, "tardir:%s, delay_ms:%ld, min_time:%f, skillinstid:%d", V3FSTR(tardir),
            delay_ms, min_time, src_skillctx_instid);
  return 0;
}

std::string CMoveComponent::DebugString() const { return data_.Debug2Json(); }

void CMoveComponent::FillPointDir(VECTOR3F *face_point) {  // 填充朝向数组
  auto pointcnt = GetPathPointCount();
  if (!face_point || IS_V3F_ZERO((*face_point))) {
    for (int i = 0; i < pointcnt - 1; ++i) {
      data_.path_point_list[i].dir =
          v3f::Normalized(data_.path_point_list[i + 1].pos - data_.path_point_list[i].pos);
    }
  } else {
    for (int i = 0; i < pointcnt - 1; ++i) {
      data_.path_point_list[i].dir = v3f::Normalized(*face_point - data_.path_point_list[i].pos);
      LogTraceM(
          LOGM_MOVE, "target move, point:%d, move dir%s, target dir%s", i,
          V3FSTR(v3f::Normalized(data_.path_point_list[i + 1].pos - data_.path_point_list[i].pos)),
          V3FSTR(v3f::Normalized(*face_point - data_.path_point_list[i].pos)));
    }
  }

  // 保证dir列表与point列表对应
  if (pointcnt == 1) {
    auto &dirpt = data_.path_point_list[0];
    auto dir = dirpt.pos - GetCoOwner()->GetPos();
    dir.z = 0;
    dirpt.dir = v3f::Normalized(dir);
  } else if (pointcnt >= 2) {
    data_.path_point_list[pointcnt - 1].dir = data_.path_point_list[pointcnt - 2].dir;
  }
}

void CMoveComponent::UpdateActorPosMark(const VECTOR3F *dest_point /* = nullptr*/) {
  auto px_com = GetOwner()->GetCom<PhysXComponent>();
  if (px_com == nullptr) {
    return;
  }
  auto temp_pos = GetDestPoint();
  if (dest_point) {
    temp_pos = *dest_point;
  }

  CMap *map = GetMapinstTable().FindMap(GetCoOwner()->GetMapGID());
  if (map) {
    // 没有移动的实体, (当前坐标) ud_pos_mask 绑定到地图场景中
    // 正在移动的实体, (终点坐标) ud_pos_mask 绑定到地图场景中
    map->UpdateActor(px_com->GetPosMark(), temp_pos);
  }
  // debug，客户端显示移动路径，不依赖于ue，且我们画连线
  auto radius = GetOwner()->GetCom<PhysXComponent>()->GetMoveRadius();
  DebugDrawSphere(*GetOwner(), WATCH_MODE_PHYSX_ACTOR, temp_pos, radius, 1,
                  PBCOLOR(128, 128, 128, 255));
}

bool CMoveComponent::IsFlying() const { return GetCurState() == MOVE_TYPE_FLY; }

int CMoveComponent::GetMapResID() const { return GetCoOwner()->GetMapResID(); }

VECTOR3F CMoveComponent::GetPos() const { return GetCoOwner()->GetPos(); }

void CMoveComponent::SetCurState(int state) {
  auto oldvalue = static_cast<MOVE_TYPE>(data_.cur_state);
  auto newvalue = static_cast<MOVE_TYPE>(state);
  LogTraceM(LOGM_MOVE, "%lu,%s, state:%s-->%s", GetOwnerid(), GetOwner()->GetObjName(),
            MOVE_TYPE_Name(oldvalue).c_str(), MOVE_TYPE_Name(newvalue).c_str());

  data_.cur_state = state;
  auto &statcom = GetCoOwner()->CommstatCom();
  switch (state) {
    case MOVE_TYPE_WALK:
      GetCoOwner()->SetAttr(hx2proto::ATTR_MOVE_VERTICAL_LOCATION, MOVE_VERTICAL_LOCATION_GROUND);
      statcom.Set(hx2proto::RES_COMMSTAT_MOVING);  // 客户端触发的，无法check，不check了
      break;
    case MOVE_TYPE_FLY:
    case MOVE_TYPE_GLIDE:
      GetCoOwner()->SetAttr(hx2proto::ATTR_MOVE_VERTICAL_LOCATION, MOVE_VERTICAL_LOCATION_AIR);
      statcom.Set(hx2proto::RES_COMMSTAT_FLY);  // 客户端触发的，无法check，不check了
      break;
    case MOVE_TYPE_SWIM:
      GetCoOwner()->SetAttr(hx2proto::ATTR_MOVE_VERTICAL_LOCATION, MOVE_VERTICAL_LOCATION_WATER);
      break;
    default:
      break;
  }
  // 后置处理
  if (oldvalue != newvalue) {
    OnCurStateChanged(oldvalue, newvalue);
  }
}

// 当前状态变化后置处理
void CMoveComponent::OnCurStateChanged(MOVE_TYPE old_state, MOVE_TYPE new_state) {
  // 潜行进入判断
  if (new_state == MOVE_TYPE_WALK || new_state == MOVE_TYPE_STOP) {
    TryStartSneaking();
  }
  // 潜行退出判断
  if (new_state != MOVE_TYPE_WALK && new_state != MOVE_TYPE_STOP) {
    StopSneaking();
  }
}

// 设置子状态
void CMoveComponent::SetSubState(int state) {
  if (data_.sub_state == state) {
    return;
  }
  auto old_state = data_.sub_state;
  data_.sub_state = state;
  OnSubStateChanged(old_state, state);
}

// 子状态变化后置处理
void CMoveComponent::OnSubStateChanged(int old_state, int new_state) {
  if (data_.cur_state == MOVE_TYPE_WALK) {                                     // 步行
    if (old_state == MoveSubType_Sprint && new_state == MoveSubType_Normal) {  // 尝试进入潜行
      TryStartSneaking();
    } else if (old_state == MoveSubType_Sprint && new_state == MoveSubType_Normal) {  // 退出潜行
      StopSneaking();
    }
  }
}

bool CMoveComponent::IsServerPathfinding() {
  auto *co = GetCoOwner();
  if (co->IsRole()) {
    return co->ToRole()->Vehicleattach().GetTouchStatus() != kVehicleTouchingNone;
  }
  // 被客户端玩家操控的战兽，并不是Server寻路
  if (co->CommstatCom().Is(RES_COMMSTAT_BE_CONTROLLED)) {
    int id = co->GetAttr(ATTR_CONTROLLED_BY_OBJ);
    return GID_TO_GID_TYPE(id) != GID_ROLE;
  }
  return true;
}

int CMoveComponent::FlyTakeOff(int height) {
  ERR_IF_FALSE(height > 0, -1, "no height");

  VECTOR3F start_pos = GetPos();
  VECTOR3F end_pos = start_pos + VECTOR3F(0, 0, height);

  // ok--
  int ret = SetSimplePath(start_pos, end_pos, MOVE_TYPE_FLY);
  ERR_IF_FALSE(ret == 0, -1, "path wrong");
  GetOwner()->SetAttr(ATTR_FLY_STATE, AI_FLY_STATE_TAKEOFF);
  GetOwner()->SetAttr(ATTR_MOVE_VERTICAL_LOCATION, MOVE_VERTICAL_LOCATION_AIR);
  LogTrace("id:%ld, start:%s, end:%s", GetOwnerid(), V3FSTR(start_pos), V3FSTR(end_pos));

  return 0;
}

int CMoveComponent::FlyCruise(VECTOR3F pos) {
  OnPath();
  VECTOR3F start_pos = GetPos();
  int ret = SetSimplePath(start_pos, pos, MOVE_TYPE_FLY);
  ERR_IF_FALSE(ret == 0, -1, "nav fail");
  GetOwner()->SetAttr(ATTR_FLY_STATE, AI_FLY_STATE_CRUISE);
  GetOwner()->SetAttr(ATTR_MOVE_VERTICAL_LOCATION, MOVE_VERTICAL_LOCATION_AIR);

  LogDebug("id:%ld start:%s, end:%s", GetOwnerid(), V3FSTR(start_pos), V3FSTR(pos));
  return 0;
}

int CMoveComponent::FlyLanding() {
  // 着陆时直接贴地，不应该依赖于配置
  OnPath();
  VECTOR3F ground_point;
  auto ret = GetCoOwner()->PhysXCom().GetGroundPos(GetPos(), ground_point, 0, 100000);
  ERR_IF_FALSE(ret, -1, "no ground point");

  VECTOR3F start_pos = GetCoOwner()->GetPos();
  ret = SetSimplePath(start_pos, ground_point + VECTOR3F(0, 0, GetCoOwner()->GetCenterGroundDist()),
                      MOVE_TYPE_FLY);
  ERR_IF_FALSE(ret == 0, -1, "nav fail");
  GetOwner()->SetAttr(ATTR_FLY_STATE, AI_FLY_STATE_LANDING);

  LogTrace("id:%lu start:%s end:%s", GetOwnerid(), V3FSTR(start_pos), V3FSTR(ground_point));
  return 0;
}

int CMoveComponent::FlyPursuit(VECTOR3F pos) {
  // 按照原来的方案，到目标的上方再降落landing
  OnPath();
  VECTOR3F start_pos = GetPos();
  int ret = SetSimplePath(start_pos, pos, MOVE_TYPE_FLY);
  ERR_IF_FALSE(ret == 0, -1, "nav fail");
  GetOwner()->SetAttr(ATTR_FLY_STATE, AI_FLY_STATE_PURSUIT);
  GetOwner()->SetAttr(ATTR_MOVE_VERTICAL_LOCATION, MOVE_VERTICAL_LOCATION_AIR);

  LogDebug("id:%ld start:%s, end:%s", GetOwnerid(), V3FSTR(start_pos), V3FSTR(pos));
  return 0;
}

int CMoveComponent::FlyDie() {
  // 非飞行状态
  if (!IsFlying()) {
    return 0;
  }

  OnPath();
  VECTOR3F ground_point;
  auto ret = GetCoOwner()->PhysXCom().GetGroundPos(GetPos(), ground_point, 0, 100000);
  ERR_IF_FALSE(ret, -1, "no ground point");

  GetOwner()->SetAttr(ATTR_FLY_STATE, AI_FLY_STATE_FALL);

  VECTOR3F start_pos = GetCoOwner()->GetPos();
  ret = SetSimplePath(start_pos, ground_point + VECTOR3F(0, 0, GetCoOwner()->GetCenterGroundDist()),
                      MOVE_TYPE_FREE_FALL);
  ERR_IF_FALSE(ret == 0, -1, "nav fail");

  LogTrace("id:%lu start:%s end:%s", GetOwnerid(), V3FSTR(start_pos), V3FSTR(ground_point));

  return 0;
}

int CMoveComponent::OnFinishPath() {
  if (IsFlying()) {
    switch (GetCurFlyState()) {
      case AI_FLY_STATE_LANDING: {
        SetCurState(MOVE_TYPE_STOP);
        GetCoOwner()->SetAttr(ATTR_MOVE_VERTICAL_LOCATION, MOVE_VERTICAL_LOCATION_GROUND);
        break;
      }
      case AI_FLY_STATE_CRUISE: {
        // 固定路径，则检查当前是不是在空中，否则设飞行状态为false
        if (!GetCoOwner()->PhysXCom().IsPointInAir(GetPos())) {
          GetOwner()->SetAttr(ATTR_MOVE_VERTICAL_LOCATION, MOVE_VERTICAL_LOCATION_GROUND);
          SetCurState(MOVE_TYPE_STOP);
          LogDebug("%d point:%s not in air, movestate:%s", GetCoOwner()->GetResid(),
                   V3FSTR(GetPos()), MOVE_TYPE_Name((MOVE_TYPE)GetCurState()).c_str());
        }
        break;
      }
      default:
        break;
    }

    // movecom内部仅做本次移动后的属性修正，不会重启新的寻路，
    // 重启新的寻路是ai层的事情，这里发送event去触发
    EVENTCONTEXT evt;
    INIT_EVT_OBJ_SIMPLE(evt, GetVisiOwner());
    *evt.mutable_fly_arrive_point() = 1;
    GetOwner()->OnEvent(evt);
  } else {
    // 不在空中才设为STOP
    SetCurState(MOVE_TYPE_STOP);
    GetCoOwner()->NavCom().SetNavReason(hx2proto::NAVI_REASON_NONE);
  }
  return 0;
}

int CMoveComponent::OnStopMoveInAir() { return 0; }
