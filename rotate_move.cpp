#include "rotate_move.h"
#include "vector3.h"

int RotateMove::SetDesiredDir(VECTOR3F dir, hx2proto::MoveReason reason, int64_t reason_para) {
  data_.desired_dir = dir;
  return 0;
}

bool RotateMove::IsRotating(ACombatActor *actor) const {
  // 如果是DetourCrowd算法控制的对象直接忽略，因为算法计算的时候已经考虑到了
  if (actor->NavCom().IsCrowdAlgoControlAgent()) {
    return false;
  }
  // 靶向移动的时候，不原地转身
  if (actor->MoveCom().IsTargetMove()) {
    return false;
  }
  if (actor->NavCom().GetNavReason() == NAVI_REASON_BT_PURSUIT ||
      actor->NavCom().GetNavReason() == NAVI_REASON_SKILL_PURSUIT) {
    return false;
  }
  float degree = v3f::GetDegree(actor->PosCom().GetDir2D(), GetTarDir2D(), false);
  if (degree < MIN_ROTATE_DEGREE) {
    return false;
  }
  const auto *modelcfg = actor->GetModelCfg();
#define MIN_ROTATE_DEGREE 2.0f
  if (modelcfg && modelcfg->minrotatedegree() >= 0.01f &&
      (degree - modelcfg->minrotatedegree()) < MIN_ROTATE_DEGREE) {
    return false;
  }
  return true;
}

void RotateMove::UpdateRotate(ACombatActor *actor, uint64_t &left_ms) {
  if (!IsRotating(actor)) {
    return;
  }

  auto curdir = actor->PosCom()->GetDir();
  auto desdir = data_.desired_dir;
  int total_degree = v3f::GetDegree(curdir, desdir, true);
  const auto *modelcfg = actor->GetModelCfg();
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
    actor->PosCom().SetDir(GetTarDir(), SetDirReason_StepRotate, false);

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

int RotateMove::DelayRotate(VECTOR3F tardir, float min_time, float speed, int64_t delay_ms,
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

int RotateMove::RotateStep(VECTOR3F target_dir, NAVI_REASON reason, float min_time, float speed,
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

int64_t RotateMove::CalcRotateTime(ACombatActor *actor, int &degree, float min_time,
                                   float speed) const {
  // debug 如果在战兽上，不转身, detour crowd算法控制的agent也不转身
  if (actor->IsOnVehicle() || actor->NavCom().IsCrowdAlgoControlAgent()) {
    return 0;
  }

  auto &poscom = actor->PosCom();

  auto curdir = poscom.GetDir();
  auto desdir = data_.desired_dir;
  degree = v3f::GetDegree(curdir, desdir, false);

  auto reason = actor->NavCom().GetNavReason();
  if (reason == NAVI_REASON_ROTATE_SKILL) {
    // skill rotate, time by speed
    float costms = degree / MAX(1, speed) * 1000;
    return MAX(costms, min_time * 1000);
  }

  // 如果配置时间为空，则不需要转身时间
  const auto *modelcfg = actor->GetModelCfg();
  ERR_IF_NULL_SIMP(modelcfg, 0);
  if (modelcfg->rotatespeed() <= 0.01f) {
    return 0;
  }

  float rotatems = 0;
  // rotate speed 单位角度/s
  rotatems = degree / modelcfg->rotatespeed() * 1000.0f;
  // 有转身时间
  if (reason == NAVI_REASON_BT_PURSUIT) {
    if (degree < 60) {
      // 小角度追击
      return 0;
    }

    // 大角度追击
    if (actor->MoveCom().IsServerPathfinding()) {
      if (!actor->IsMonster()) {
        return 0;
      }
      const RESMONSTERDEF *res = actor->ToMonster()->FindRes();
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