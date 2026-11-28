/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <super_core/super_planner.h>
#include <memory>
#include <cstdlib>
#include <sstream>
#include <super_utils/scope_timer.hpp>
#include <fmt/color.h>

using namespace super_utils;

namespace super_planner {
    SuperPlanner::SuperPlanner
            (const std::string &cfg_path,
             const ros_interface::RosInterface::Ptr &ros_ptr,
             const rog_map::ROGMapROS::Ptr &map_ptr
            ) : cfg_(Config(cfg_path)), ros_ptr_(ros_ptr), map_ptr_(map_ptr) {

        ros_ptr_->setResolution(cfg_.resolution);
        ros_ptr_->setVisualizationEn(cfg_.visualization_en);
        exp_traj_opt_ = std::make_shared<traj_opt::ExpTrajOpt>(cfg_.exp_traj_cfg, ros_ptr_);
        back_traj_opt_ = std::make_shared<traj_opt::BackupTrajOpt>(cfg_.back_traj_cfg, ros_ptr_);
        yaw_traj_opt_ = std::make_shared<traj_opt::YawTrajOpt>(cfg_.yaw_dot_max);
        const auto &rog_map_cfg = map_ptr_->getMapConfig();
        astar_ptr_ = std::make_shared<path_search::Astar>(cfg_path, ros_ptr_, map_ptr_);
        cg_ptr_ = std::make_shared<CorridorGenerator>(ros_ptr_, map_ptr_, cfg_.corridor_bound_dis,
                                                      cfg_.corridor_line_max_length,
                                                      cfg_.resolution, rog_map_cfg.virtual_ground_height,
                                                      rog_map_cfg.virtual_ceil_height,
                                                      cfg_.robot_r,
                                                      cfg_.obs_skip_num,
                                                      cfg_.iris_iter_num);
        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);


        time_consuming_.resize(8);

        /// WHTF-CONFIG-LOAD (Phase 1, 2026-11-27): print the flags that Phase 1 relies on, so a flight log
        /// proves the config file was really loaded (a silent "flag did nothing" is the worst failure mode).
        std::cout << " -- [SUPER] CONFIG: general_bc=" << (cfg_.general_bc_en ? 1 : 0)
                  << " keep_old_goal_guide=" << (cfg_.keep_old_goal_guide ? 1 : 0)
                  << " handover=" << cfg_.handover_policy
                  << " guide_cache=" << (cfg_.guide_cache_en ? 1 : 0)
                  << " hot_start=" << (cfg_.hot_start_en ? 1 : 0)
                  << " hot_ratio=" << cfg_.hot_start_vel_ratio
                  << " pva_tail_len=" << cfg_.pva_tail_len
                  << " pva_tail_adaptive=" << (cfg_.pva_tail_adaptive ? 1 : 0)
                  << " receding_dis=" << cfg_.receding_dis
                  << " max_vel=" << cfg_.exp_traj_cfg.max_vel << std::endl;

        robot_state_.rcv = false;
        planner_process_start_WT_ = ros_ptr_->getSimTime();
        fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI,
                                                    -1.0,
                                                    -35.0,
                                                    35.0);

        const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);
        astar_ptr_->setFineInfNeighbors(neighbor_step);
    }

    RET_CODE
    SuperPlanner::PlanFromRest(const Vec3f &goal_p,
                               const double &goal_yaw,
                               const bool &new_goal,
                               const Vec3f &goal_v,
                               const Vec3f &goal_a,
                               const Vec3f &via_p,
                               const bool &has_via) {
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
        if (robot_state_.rcv == false) {
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest]: No odom, force return.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_ODOM);
            return FAILED;
        }
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.goal_v = goal_v;
        gi_.goal_a = goal_a;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        gi_.via_p = via_p;
        gi_.has_via = has_via;
        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("viz goal path", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) First, shift the start_point to free space.
        Vec3f local_star_pt;
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0)) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        latest_replan.setLocalStartP(local_star_pt);

        /// 2) Generate Exp traj
        ExpTraj exp_traj_info;
        BackupTraj back_traj_info;
        last_exp_traj_info_.setEmpty();
        local_start_p_ = local_star_pt;
        RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
        //GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        if (exp_ret_code == FAILED) {
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                           RET_CODE_STR[exp_ret_code].c_str());
            return FAILED;
        } else {
            ros_ptr_->info(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        auto log_commit = [&](const char *where, int backup_rc) {
            double T = cmd_traj_info_.getTotalDuration();
            Vec3f e = cmd_traj_info_.getPos(T);
            std::cout << " -- [SUPER] COMMIT sim=" << ros_ptr_->getSimTime() << " " << where << " total=" << T
                      << " end=(" << e.transpose() << ") backup_rc=" << backup_rc << std::endl;
        };
        if (!cfg_.backup_traj_en) {
            // 2026-11-28 review P0-2: backup_traj_en used to be a DEAD parameter; the backup was
            // optimised unconditionally and, on success, the committed trajectory was truncated to
            // "old prefix + brake segment", which is what lets a failed replan chain leave a short
            // committed trajectory that expires mid-leg (rest2rest from a moving state).
            cmd_traj_info_.setTrajectory(exp_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_NO_BACKUP);
            log_commit("rest2rest(no-backup)", -1);
            return SUCCESS;
        }
        back_traj_info.setEmpty();
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);;

        if (back_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;
            log_commit("rest2rest(backup)", static_cast<int>(back_ret_code));

            // For visualization
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
                latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_WITH_BACKUP);
            }

            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == NO_NEED) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory Finish or NO_NEED.");
            }
            robot_on_backup_traj_ = false;
            cmd_traj_info_.setTrajectory(exp_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;
            log_commit("rest2rest(free)", static_cast<int>(back_ret_code));

            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            {
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory return [{}], force return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }


    RET_CODE
    SuperPlanner::ReplanOnce(const Vec3f &goal_p,
                             const double &goal_yaw,
                             const bool &new_goal,
                             const Vec3f &goal_v,
                             const Vec3f &goal_a,
                             const Vec3f &via_p,
                             const bool &has_via) {
        TimeConsuming replan_total_t("ReplanOnce", false);
        std::lock_guard<std::mutex> guard(replan_lock_);

        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.goal_v = goal_v;
        gi_.goal_a = goal_a;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        gi_.via_p = via_p;
        gi_.has_via = has_via;
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);

        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) Replan EXP traj
        ExpTraj exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
        time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            return FAILED;
        } else if (exp_ret_code == NEW_TRAJ) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Last epx traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED) {
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need to replan a new exp traj, use last one.");
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        auto log_commit = [&](const char *where, int backup_rc) {
            double T = cmd_traj_info_.getTotalDuration();
            Vec3f e = cmd_traj_info_.getPos(T);
            std::cout << " -- [SUPER] COMMIT sim=" << ros_ptr_->getSimTime() << " " << where << " total=" << T
                      << " end=(" << e.transpose() << ") backup_rc=" << backup_rc << std::endl;
        };
        if (!cfg_.backup_traj_en) {
            cmd_traj_info_.setTrajectory(exp_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            log_commit("replan(no-backup)", -1);
            return SUCCESS;
        }
        BackupTraj back_traj_info;
        // 2）生成back轨迹
        TimeConsuming t_back("t_back", false);
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);
        time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

        {
            ft += time_consuming_[EPX_TRAJ_FRONTEND] + time_consuming_[BACK_TRAJ_FRONTEND];
            ft_cnt++;
            bt += time_consuming_[BACK_TRAJ_OPT] + time_consuming_[EXP_TRAJ_OPT];
            bt_cnt++;
        }

        double replan_dt = replan_total_t.stop();
        /// WHTF-REPLAN-TIME-LIMIT (patch 10).  This used to compare a WALL-CLOCK duration against
        /// `replan_forward_dt`, which is the TRAJECTORY-TIME lookahead -- one number, two meanings.
        /// At 0.1 s that leaves a 0.09 s budget, and MEASURED on missionA-kwC-01-01 this line alone
        /// refused 123 valid plans at replan dt 0.108-0.135 s.  Every refusal throws away a plan
        /// that was already optimised and collision-checked, so the committed trajectory is never
        /// updated, the published reference freezes, and the aircraft stops dead mid-leg.  The
        /// budget is now its own parameter and generous: a late plan is still a usable plan at
        /// 10 Hz, and patch 9 keeps publishing the committed trajectory while the optimizer runs.
        if (replan_dt > cfg_.replan_time_limit) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan too slow ({} s > {} s), check host load.",
                           replan_dt, cfg_.replan_time_limit);
            return FAILED;
        }

        if (back_ret_code == SUCCESS) {
            cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;
            log_commit("replan(backup)", static_cast<int>(back_ret_code));

            {
                // For visualization
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            latest_replan.setRetCode(SUPER_SUCCESS_WITH_BACKUP);
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) {
            // 这次生成backup轨迹的点没有意义,
            robot_on_backup_traj_ = false;
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();

            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            // Which means the exp traj is all in known free, no need for backup traj
            cmd_traj_info_.setTrajectory(exp_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;
            log_commit("replan(free)", static_cast<int>(back_ret_code));

            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: generateBackupTrajectory return {}, replan Failed return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }

    void SuperPlanner::getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish) {
        // 2026-11-28 review P0: this runs on the 100 Hz publish thread while the exec thread writes the
        // committed trajectory (setTrajectory); reading Trajectory::pieces unlocked is the same class of
        // race that crashed SITL twice before patch18.
        cmd_traj_info_.lock();
        double eval_t = (ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime());
        traj_finish = false;
        double total_dur = cmd_traj_info_.getTotalDuration();
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        start_WT_pos = cmd_traj_info_.getStartWallTime();
        if (cmd_traj_info_.backupTrajAvilibale() && eval_t > cmd_traj_info_.getBackupTrajStartTT()) {
            robot_on_backup_traj_ = true;
        } else {
            robot_on_backup_traj_ = false;
        }
        cmd_traj_info_.unlock();
    }

    Trajectory SuperPlanner::getCommittedPositionTrajectory() {
        return cmd_traj_info_.posTraj();
    }

    Trajectory SuperPlanner::getCommittedYawTrajectory() {
        return cmd_traj_info_.yawTraj();
    }


    void SuperPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                             double &yaw,
                                             double &yaw_dot,
                                             bool &on_backup_traj,
                                             bool &traj_finish) {
        cmd_traj_info_.lock();
        const double &cur_t = ros_ptr_->getSimTime();
        const double &cmd_start_WT = cmd_traj_info_.getStartWallTime();
//        const bool &backup_avilibale = cmd_traj_info_.backupTrajAvilibale();
//        const double &backup_start_TT = cmd_traj_info_.getBackupTrajStartTT();
        const double &total_dur = cmd_traj_info_.getTotalDuration();

        traj_finish = (cur_t - cmd_start_WT) > total_dur;
        const double &eval_t = traj_finish ? total_dur : (cur_t - cmd_start_WT);

//        bool last_round_robot_on_backup_traj = robot_on_backup_traj_;
        robot_on_backup_traj_ = cmd_traj_info_.isTTOnBackupTraj(eval_t);
        on_backup_traj = robot_on_backup_traj_;

        pvaj = cmd_traj_info_.posTraj().getState(eval_t);


        /// Get Yaw planning
        static double last_yaw = robot_state_.yaw;

        yaw = cmd_traj_info_.getYaw((eval_t))[0];
        yaw_dot = cmd_traj_info_.getYawRate((eval_t))[0];

        if (isnan(yaw)) {
            yaw = last_yaw;
            yaw_dot = 0;
        } else {
            last_yaw = yaw;
        }
        if (isnan(yaw_dot)) {
            yaw_dot = 0;
        }

//        if (last_round_robot_on_backup_traj != robot_on_backup_traj_) {
//            if (last_round_robot_on_backup_traj) {
//                ros_ptr_->info(" -- [CMD] Emergency Stop End ========================");
//            } else {
//                ros_ptr_->info(" -- [CMD] Emergency Stop Start ========================");
//            }
//        }

//        double cur_yaw = geometry_utils::get_yaw_from_quaternion(robot_state_.q);
        cmd_traj_info_.unlock();
    }


    void SuperPlanner::getModuleTimeConsuming(vector<double> &time) {
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }


    RET_CODE SuperPlanner::generateExpTraj(ExpTraj &last_exp_traj_info, ExpTraj &out_exp_traj_info) {
        /* 1) Log the exp traj frontend time*/
        TimeConsuming t_exp_frontend("t_exp_frontend", false);

        // use hot init or not, just prepare a guide path, a guide t, init and fina state and sfc for exp traj opt
        StatePVAJ pos_init_state, pos_fina_state;
        bool used_general_bc = false;   ///< WHTF-GENERAL-BC (patch 17) -- for the zero-start retry below
        PolytopeVec sfc;
        vec_Vec3f guide_path;
        bool astar_used = false;   ///< WHTF-REPLAN-DEBUG: did this replan run A*?
        // the guide_stamp saves a TT
        vector<double> guide_stamp;
        double guide_path_end_vel{0.0};
        int reserve_size = cfg_.planning_horizon / cfg_.resolution * 1.2;
        guide_path.reserve(reserve_size);
        guide_stamp.reserve(reserve_size);

        Vec4f init_yaw{robot_state_.yaw, 0, 0, 0};
        Vec4f fina_yaw{0, 0, 0, 0};


        // alias for last_exp_traj_info
        Trajectory guide_pos_traj, guide_yaw_traj, last_exp_traj;

        // record the wall time (WT) and the trajectory time (TT) at the start of the replan.
        const double replan_process_start_WT = ros_ptr_->getSimTime();
        double replan_process_start_TT, replan_state_TT;

        /* 2) Check last exp traj */
        if (last_exp_traj_info.empty()) {
            /* 2.1) Perform rest2rest exp traj generation */
            // just skip the first part of the guide trajectory
            pos_init_state.setZero();
            pos_init_state.col(0) = local_start_p_;
            /// HOT START (super_planner/hot_start_en).  A rest2rest plan is correct the first time a
            /// goal arrives; it is WRONG when it is re-issued because the committed trajectory ran
            /// out while the vehicle was still flying.  That is the normal case at a waypoint:
            /// fsm_ros2.hpp's getPoseFromTraj() sends the FSM to GENERATE_TRAJ on `traj_finish_`
            /// unless it is already within 0.1 m of the goal, so the next plan starts where the
            /// last one ended -- at speed -- and zeroing col(1) makes the reference brake to 0 and
            /// then accelerate again (measured: 29 full stops in 80 s, 20 % of the route below
            /// 0.5 m/s, missionA-pg-01).  Injecting the measured velocity keeps the handover
            /// continuous.  Everything else -- SFC, A*, MINCO, the terminal rest at the goal -- is
            /// untouched.
            if (cfg_.general_bc_en) {
                /// WHTF-GENERAL-BC (patch 17).  The vehicle is a CONTINUOUS system: at a handover it is
                /// moving (and accelerating) while the planner starts a fresh MINCO problem.  Hand the
                /// optimiser the measured state so the new trajectory starts on the same physical state
                /// -- no brake-to-zero, no reference cusp.  The caps are the planner's own envelope.
                Vec3f v0 = robot_state_.v;
                const double vn0 = v0.norm();
                const double v_cap = cfg_.exp_traj_cfg.max_vel;
                if (vn0 > v_cap && vn0 > 1e-6) { v0 *= v_cap / vn0; }
                if (vn0 > 1e-3) { pos_init_state.col(1) = v0; }
                Vec3f a0 = robot_state_.a;
                const double an0 = a0.norm();
                const double a_cap = cfg_.exp_traj_cfg.max_acc;
                if (an0 > a_cap && an0 > 1e-6) { a0 *= a_cap / an0; }
                if (an0 > 1e-3) { pos_init_state.col(2) = a0; }
                used_general_bc = true;
                cout << " -- [SUPER] GENERAL-BC: start from measured state" << endl;
            } else if (cfg_.hot_start_en) {
                Vec3f v0 = robot_state_.v;
                const double vn0 = v0.norm();
                const double v_cap = cfg_.exp_traj_cfg.max_vel * cfg_.hot_start_vel_ratio;   // WHTF-HOTRATIO
                if (vn0 > v_cap && vn0 > 1e-6) {
                    v0 *= v_cap / vn0;
                }
                const double stop_dis = v_cap * v_cap / (2.0 * cfg_.exp_traj_cfg.max_acc) + 0.3;
                if (vn0 > 0.05 && (gi_.goal_p - local_start_p_).norm() > stop_dis) {
                    pos_init_state.col(1) = v0;
                }
            }
            replan_process_start_TT = -1;
            replan_state_TT = -1;
        } else {
            guide_pos_traj = cmd_traj_info_.posTraj(); // last_exp_traj;
            guide_yaw_traj = cmd_traj_info_.yawTraj(); //last_exp_traj_info.exp_yaw_traj;
            last_exp_traj = last_exp_traj_info.posTraj();

            replan_process_start_TT = replan_process_start_WT - last_exp_traj.start_WT;
            replan_state_TT = replan_process_start_TT + cfg_.replan_forward_dt;
            /* 2.2) Perform collision check on last exp traj*/
            vector<TimePosPair> last_exp_traj_time_pos;
            vector<double> last_exp_traj_vel;


            // check early exit condition
            // 1) if the replan state is beyond the last cmd traj, return NO_NEED
            if (replan_state_TT >= cmd_traj_info_.getTotalDuration()) {
                out_exp_traj_info = last_exp_traj_info;

                if (robot_on_backup_traj_) {
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                }

                if (cfg_.print_log) {
                    ros_ptr_->warn(
                            " -- [generateExpTraj] replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                }
                return NO_NEED;
            }

            if (!last_exp_traj_info.empty()) {
                if (replan_state_TT >= last_exp_traj.getTotalDuration()) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [generateExpTraj] replan_state_TT >= last_exp_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                /// 1) Check a series of early termination conditions.
                if (!gi_.new_goal && last_exp_traj_info.getSFCSize() == 1 && last_exp_traj_info.connectedToGoal()) {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, last exp have only one corridor and connected to goal return NONEED.");
                    }

                    out_exp_traj_info = last_exp_traj_info;
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                if (!gi_.new_goal &&
                    (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3) {
                    // Return if the traj close to goal
                    out_exp_traj_info = last_exp_traj_info;
                    out_exp_traj_info.setGoalConnectedFlag(true);

                    ros_ptr_->warn(" -- [SUPER] Replan, close to goal and return NONEED.");
                    if (robot_on_backup_traj_) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }
            }
            /// Ready for replan.
            out_exp_traj_info.setGoalConnectedFlag(false);

            // * 2) Check if in backup trajectory. While in backup trajectory,
            // *    the guide trajectory should be a part of cmd trajectory.
            // TODO: Why cannot directly replan on cmd traj? 241121

            // * 3) Perform collision check on the guide trajectory.
            // TODO 0929 critical change for hot init.
            double eval_t = replan_state_TT; //replan_process_start_TT;
            double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();

            Vec3f temp_pt, last_sample_pt;
            last_exp_traj_time_pos.clear();
            last_exp_traj_info.setWholeTrajKnownFreeFlag(true);
            last_sample_pt = guide_pos_traj.getPos(eval_t);
            eval_t += cfg_.sample_traj_dt;
            // * 4) 记录replan点在evaluated_pts上的id
            int replan_id = -1;
            for (; eval_t < guide_pos_traj_total_time; eval_t += cfg_.sample_traj_dt) {
                temp_pt = guide_pos_traj.getPos(eval_t);
                if ((temp_pt - last_sample_pt).norm() < cfg_.resolution * 0.8) {
                    continue;
                }

                rog_map::GridType temp_grid = map_ptr_->getInfGridType(temp_pt);

                if (temp_grid == rog_map::GridType::OCCUPIED || temp_grid == rog_map::GridType::OUT_OF_MAP) {
                    last_exp_traj_info.setWholeTrajKnownFreeFlag(false);
                    break;
                }
                if (eval_t > replan_state_TT && replan_id == -1) {
                    replan_id = last_exp_traj_time_pos.size();
                }
                last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
                last_exp_traj_vel.emplace_back(guide_pos_traj.getVel(eval_t).norm());
                last_sample_pt = temp_pt;
            }


            // * 6) Decide where to split the original exp trajecory and re-plan a new one with an A*,
            // *    If the whole trajectory if free,  the whole trajectory should be receding and if not, or a new goal
            // *    is given, we should only receiding a small distance and replan new trajectory ASAP
            double split_dis = cfg_.receding_dis;
            /// WHTF-KEEP-OLD-GOAL (patch 22, plan A, 2026-11-27).  For a NEW GOAL, drop no part of the
            /// committed trajectory: keep it up to its END (the previous corner) so the corner is a
            /// guide point of the new trajectory and the A* starts AT the corner instead of 1 m before
            /// it.  The corner stays inside the same keepout, so W1b is untouched.
            if (cfg_.receding_dis > 0.0) {
                if (!gi_.new_goal) {
                    // UPSTREAM behaviour for an ordinary replan: the whole committed trajectory is the
                    // guide when it is known free (round33 had changed this to 12 m for every replan).
                    if (last_exp_traj_info.wholeTrajKnownFree()) {
                        split_dis = std::numeric_limits<double>::max();
                    }
                } else {
                    // HANDOVER (defer or splice): keep the old trajectory -- including the corner it ends
                    // at -- as the guide prefix, so that corner is an INTERIOR point instead of being cut
                    // by a fresh A*; leave the A* a usable horizon (planning_horizon - 4 m).
                    // 2026-11-28 audit: this MUST NOT be gated by wholeTrajKnownFree(), which checks the
                    // committed trajectory against the 0.2 m INFLATED map while MINCO only guarantees
                    // robot_r=0.10 m -- the gate was almost always false, so round42's prefix retention
                    // silently fell back to split=1.0 m and A* cut the corner (R1a's real failure mode).
                    // If the old prefix does hit a NEW obstacle, MINCO/SFC rejects the plan and the
                    // committed trajectory is kept (FAILED), which is the safe outcome.
                    split_dis = std::max(cfg_.receding_dis, cfg_.planning_horizon - 4.0);
                }
            }
            if (std::getenv("WHTF_REPLAN_DEBUG") != nullptr) {
                std::ostringstream ros_;
                ros_.setf(std::ios::fixed); ros_.precision(3);
                ros_ << "REPLANDBG wall=" << ros_ptr_->getSimTime()
                     << " new_goal=" << (gi_.new_goal ? 1 : 0)
                     << " keep=" << (cfg_.keep_old_goal_guide ? 1 : 0)
                     << " whole_free=" << (last_exp_traj_info.wholeTrajKnownFree() ? 1 : 0)
                     << " split=" << (split_dis > 1e5 ? -1.0 : split_dis)
                     << " remain=" << (cmd_traj_info_.getTotalDuration() - replan_state_TT);
                std::cout << ros_.str() << std::endl;
            }


            // * 7）Begin replan process, first get the replan state from the committed trajectory.
            if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            // * Generate guide path with time stampe, for hot trajectory initialization
            // * the guide stamp is time from the replan start t
            guide_stamp.clear();
            guide_path.clear();
            if (split_dis <= 0 || last_exp_traj_time_pos.empty()) {
                /// No need receding, just path search.
                guide_path.push_back(pos_init_state.col(0));
                guide_stamp.push_back(0.0);
                last_exp_traj_time_pos.clear();
                last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                guide_path_end_vel = robot_state_.v.norm();
            } else {
                temp_pt = last_exp_traj_time_pos.back().second;
                // * 8) Pop all evaluated pts after the sampled point.
                while (map_ptr_->isOccupiedInflate(temp_pt) ||
                       (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
                    last_exp_traj_time_pos.pop_back();
                    last_exp_traj_vel.pop_back();
                    if (last_exp_traj_time_pos.empty()) {
                        ros_ptr_->warn(" -- [SUPER] WARN, all traj is collide in INF2");
                        break;
                    }
                    temp_pt = last_exp_traj_time_pos.back().second;
                }
                if (!last_exp_traj_time_pos.empty()) {
                    for (long unsigned int i = 0; i < last_exp_traj_time_pos.size(); i++) {
                        guide_path.push_back(last_exp_traj_time_pos[i].second);
                        guide_stamp.push_back(last_exp_traj_time_pos[i].first - last_exp_traj_time_pos.front().first);
                        guide_path_end_vel = last_exp_traj_vel[i];
                    }
                } else {
                    guide_path.push_back(pos_init_state.col(0));
                    guide_stamp.push_back(0.0);
                    last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                    guide_path_end_vel = robot_state_.v.norm();
                }
            }
        }

        // second, geometry part of the guide path
        ///=================The Second Part of Guide Path ================================================

        double guide_path_length = geometry_utils::computePathLength(guide_path);
        double temp_horizon = cfg_.planning_horizon - guide_path_length;

        vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        vector<int> sfc_waypoint_ids;

        if (guide_path.empty() ||
            ((guide_path.front() - pos_init_state.col(0)).norm() > 1e-2)) {
            guide_path.insert(guide_path.begin(), pos_init_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }

        // ★ WHTF-VIA (2026-11-28) ★  A new goal may carry a MANDATORY waypoint (the pacer's in-hand
        // corner).  The 0-flight audit proved that "keep the old trajectory as the prefix" does NOT
        // guarantee that corner (the old committed trajectory may already be aiming one corner further),
        // so the only robust fix is to make the A*/guide pass through it explicitly:
        //   guide = prefix (old trajectory if available) -> A*(prefix end .. via) -> A*(via .. goal)
        auto append_astar = [&](const Vec3f &target, double horizon) -> bool {
            if (horizon <= cfg_.resolution * 2) {
                return true;
            }
            if ((guide_path.back() - target).norm() < cfg_.resolution * 5) {
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - target).norm() / cfg_.exp_traj_cfg.max_vel);
                guide_path.push_back(target);
                return true;
            }
            vec_Vec3f new_path;
            astar_used = true;
            if (!PathSearch(guide_path.back(), target, horizon, new_path)) {
                ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                return false;
            }
            if (new_path.size() < 2) {
                ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                return false;
            }
            double total_dis{0.0};
            vector<double> dis(new_path.size());
            Vec3f last_p = new_path.back();
            for (int i = static_cast<int>(new_path.size()) - 2; i >= 0; i--) {
                total_dis += (new_path[i] - last_p).norm();
                dis[i + 1] = total_dis;
                last_p = new_path[i];
            }
            total_dis += (new_path.front() - guide_path.back()).norm();
            dis[0] = total_dis;
            vector<double> stamps(new_path.size(), 0), dt(new_path.size(), 0);
            double last_stamp = 0;
            for (int i = static_cast<int>(dis.size()) - 1; i >= 0; i--) {
                double vel;
                geometry_utils::simplePMTimeAllocator(cfg_.exp_traj_cfg.max_acc, cfg_.exp_traj_cfg.max_vel,
                                                      guide_path_end_vel, total_dis, dis[i], stamps[i], vel);
                dt[i] = stamps[i] - last_stamp;
                last_stamp = stamps[i];
            }
            double time_stamp = guide_stamp.back();
            for (long unsigned int i = 1; i < new_path.size(); i++) {
                time_stamp += dt[i];
                guide_path.emplace_back(new_path[i]);
                guide_stamp.emplace_back(time_stamp);
            }
            // the next segment starts with the velocity at the end of this one (approx: max_vel)
            guide_path_end_vel = std::min(cfg_.exp_traj_cfg.max_vel, std::max(0.3, guide_path_end_vel));
            return true;
        };

        if (temp_horizon > cfg_.resolution * 2) {
            if (gi_.has_via) {
                // Does the existing guide ALREADY pass the via?  (the old committed trajectory often runs
                // through the in-hand corner, in which case adding an A* leg to it would double back and
                // can crash the corridor stage.)
                double dmin = 1e9;
                for (const auto &gp : guide_path) {
                    dmin = std::min(dmin, (gp - gi_.via_p).norm());
                }
                const double need = (guide_path.back() - gi_.via_p).norm()
                                    + (gi_.via_p - gi_.goal_p).norm();
                if (dmin < 0.5) {
                    std::cout << " -- [SUPER] VIA: prefix already passes it (dmin " << dmin
                              << " m), no extra segment" << std::endl;
                } else if (need > cfg_.planning_horizon - 1.0) {
                    // The via would consume the whole horizon; skipping it keeps the planner alive.  The
                    // thin layer logs DISPATCH-VIA-SKIP and can fall back to the old behaviour.
                    ros_ptr_->warn(" -- [SUPER] VIA: skipped (prefix->via->goal needs {:.2f} m > horizon {:.2f})",
                                   need, cfg_.planning_horizon);
                } else {
                    std::cout << " -- [SUPER] VIA: guide must pass " << gi_.via_p.transpose()
                              << " (prefix end " << guide_path.back().transpose() << ")" << std::endl;
                    if (!append_astar(gi_.via_p, temp_horizon)) {
                        return FAILED;
                    }
                    guide_path_length = geometry_utils::computePathLength(guide_path);
                    temp_horizon = cfg_.planning_horizon - guide_path_length;
                    if (temp_horizon <= cfg_.resolution * 2) {
                        ros_ptr_->warn(" -- [SUPER] VIA: no horizon left after the via segment");
                        return FAILED;
                    }
                }
            }
            if (!append_astar(gi_.goal_p, temp_horizon)) {
                return FAILED;
            }
        }
        // sanitize: the corridor generator throws on degenerate polylines (duplicate consecutive points,
        // zero-length segments).  A via segment can create such a point; drop them before the SFC stage.
        {
            vec_Vec3f sanitized;
            vector<double> sanitized_stamp;
            for (size_t i = 0; i < guide_path.size(); ++i) {
                if (!sanitized.empty() && (guide_path[i] - sanitized.back()).norm() < 1e-3) {
                    continue;
                }
                sanitized.push_back(guide_path[i]);
                sanitized_stamp.push_back(i < guide_stamp.size() ? guide_stamp[i] : 0.0);
            }
            guide_path = sanitized;
            guide_stamp = sanitized_stamp;
        }

        /// WHTF-GUIDE-CACHE (Phase 1 round 14, 2026-11-27): Nav2 CheckIfNewPathNeeded-style path reuse.
        /// MEASURED (guide_flip.py, 129a/130a/131a): the guide polyline changes at 7.3-7.5 Hz with a base
        /// jump p90 of 0.36-0.40 m and 105-116 jumps >0.35 m per sortie - the A* tail is re-chosen every
        /// replan.  If the goal has not moved, the cached guide is still collision-free on the SAME
        /// inflated map and it is younger than guide_cache_max_age, reuse it (cropped from the current
        /// position); only fall back to a fresh A* when one of those fails.
        if (cfg_.guide_cache_en && guide_path.size() >= 3) {
            static vec_Vec3f cache_guide;
            static Vec3f cache_goal(9999, 9999, 9999);
            static double cache_wall = -1e9;
            const double now_wall = ros_ptr_->getSimTime();
            bool reused = false;
            if (!cache_guide.empty() && (gi_.goal_p - cache_goal).norm() < 0.3 &&
                (now_wall - cache_wall) <= cfg_.guide_cache_max_age) {
                bool free_ok = true;
                for (size_t j = 0; j < cache_guide.size() && free_ok; ++j) {
                    if (map_ptr_->isOccupiedInflate(cache_guide[j])) { free_ok = false; }
                    if (j + 1 < cache_guide.size()) {
                        const Vec3f m = (cache_guide[j] + cache_guide[j + 1]) * 0.5f;
                        if (map_ptr_->isOccupiedInflate(m)) { free_ok = false; }
                    }
                }
                if (free_ok) {
                    const Vec3f start = guide_path.front();
                    double dbest = 1e9;
                    size_t jbest = 0;
                    for (size_t j = 0; j < cache_guide.size(); ++j) {
                        const double dd = (cache_guide[j] - start).norm();
                        if (dd < dbest) { dbest = dd; jbest = j; }
                    }
                    if (dbest < 0.8) {
                        vec_Vec3f g2;
                        g2.reserve(cache_guide.size() - jbest + 2);
                        g2.push_back(start);
                        for (size_t j = jbest; j < cache_guide.size(); ++j) { g2.push_back(cache_guide[j]); }
                        if ((g2.back() - gi_.goal_p).norm() > 0.05) { g2.push_back(gi_.goal_p); }
                        guide_path = g2;
                        guide_stamp.clear();
                        double tt = 0.0;
                        const double vr = std::max(cfg_.exp_traj_cfg.max_vel, 1e-3);
                        for (size_t i = 0; i < guide_path.size(); ++i) {
                            if (i > 0) { tt += (guide_path[i] - guide_path[i - 1]).norm() / vr; }
                            guide_stamp.emplace_back(tt);
                        }
                        reused = true;
                        std::cout << " -- [SUPER] GUIDE-CACHE: reused " << guide_path.size()
                                  << " pts (age " << (now_wall - cache_wall) << " s)" << std::endl;
                    }
                }
            }
            if (!reused) {
                cache_guide = guide_path;
                cache_goal = gi_.goal_p;
                cache_wall = now_wall;
            }
        }
        guide_path_length = geometry_utils::computePathLength(guide_path);

        const bool connected_goal = (guide_path.back().head(2) - gi_.goal_p.head(2)).norm() < cfg_.resolution * 2;
        out_exp_traj_info.setGoalConnectedFlag(connected_goal);

        /// WHTF-PVA-TAIL (patch 15, 2026-11-26).  A PVA goal carries a terminal velocity, but SUPER's
        /// trajectory endpoint is `guide_path.back()`, which is the goal itself -- so MINCO has to finish
        /// AT the corner and the reference collapses there (MEASURED phase9f: command 0.07-0.29 m/s at
        /// every corner, ~3 s dwell, 0.2-0.76 m back-and-forth).  Append a short straight tail along the
        /// terminal velocity and re-point the terminal position at the tail end: the corner remains a
        /// guide/corridor interior point (so the waypoint is still passed), the endpoint moves past it.
        if (connected_goal && cfg_.goal_pva_en && cfg_.pva_tail_len > 0.0 && !guide_path.empty()) {
            Vec3f gv(gi_.goal_v[0], gi_.goal_v[1], 0.0);
            const double gvn = gv.norm();
            if (gvn > 0.10) {
                const Vec3f tail_start = guide_path.back();
                const double stamp0 = guide_stamp.empty() ? 0.0 : guide_stamp.back();
                const double v_ref = std::max(cfg_.exp_traj_cfg.max_vel, 1e-3);
                Vec3f u = gv / gvn;
                Vec3f u_in(1.0, 0.0, 0.0);
                if (guide_path.size() >= 2) {
                    const Vec3f a = guide_path[guide_path.size() - 2];
                    Vec3f d = tail_start - a;
                    d.z() = 0.0;
                    const double dn = d.norm();
                    if (dn > 1e-6) { u_in = d / dn; }
                }
                double tail_len = cfg_.pva_tail_len;
                if (cfg_.pva_tail_adaptive) {
                    /// WHTF-ADAPTIVE-TAIL (patch 20, 2026-11-27).  The outgoing direction is not always the
                    /// free direction in this forest (c2 has a pillar 1.0 m past the corner).  Sample 9
                    /// directions in +-60 deg, march the inflated map in 5 cm steps, pick the longest
                    /// KNOWN_FREE run with a small penalty for deviating from goal_v.  The corner remains
                    /// the A* endpoint / guide interior point; only the run-out after it moves.
                    double best_score = -1e9, best_free = 0.0;
                    Vec3f best_dir = u;
                    for (int k = -4; k <= 4; ++k) {
                        const double th = static_cast<double>(k) * M_PI / 12.0;   // +-60 deg, 15 deg steps
                        Vec3f cd(std::cos(th) * u.x() - std::sin(th) * u.y(),
                                 std::sin(th) * u.x() + std::cos(th) * u.y(), 0.0);
                        double fre = 0.0;
                        for (double dd = 0.15; dd <= cfg_.pva_tail_len + 1e-9; dd += 0.05) {
                            const Vec3f p = tail_start + cd * dd;
                            if (map_ptr_->getInfGridType(p) != rog_map::GridType::KNOWN_FREE) { break; }
                            fre = dd;
                        }
                        const double score = fre - 0.02 * std::abs(static_cast<double>(k));
                        if (score > best_score) { best_score = score; best_dir = cd; best_free = fre; }
                    }
                    if (best_free >= 0.35) {
                        tail_len = std::min(cfg_.pva_tail_len, best_free - 0.05);
                        u = best_dir;
                        gi_.goal_v[0] = static_cast<float>(u.x() * gvn);
                        gi_.goal_v[1] = static_cast<float>(u.y() * gvn);
                        gi_.goal_v[2] = 0.0f;
                        if (cfg_.print_log) {
                            cout << " -- [SUPER] ADAPTIVE-TAIL: free run " << best_free
                                 << " m, tail " << tail_len << " m, dir (" << u.x() << "," << u.y()
                                 << ")" << endl;
                        }
                    } else {
                        tail_len = 0.0;
                    }
                }
                if (tail_len > 0.05) {
                    Vec3f u_mid = u_in + u;
                    u_mid.z() = 0.0;
                    const double unm = u_mid.norm();
                    if (unm > 1e-6) { u_mid = u_mid / unm; } else { u_mid = u; }
                    const double d1 = 0.35 * tail_len;
                    const double a2 = 0.70 * tail_len;
                    const double d3 = tail_len;
                    guide_path.emplace_back(tail_start + u_mid * d1);
                    guide_stamp.emplace_back(stamp0 + d1 / v_ref);
                    guide_path.emplace_back(tail_start + u * a2);
                    guide_stamp.emplace_back(stamp0 + a2 / v_ref);
                    guide_path.emplace_back(tail_start + u * d3);
                    guide_stamp.emplace_back(stamp0 + d3 / v_ref);
                    if (cfg_.print_log) {
                        ros_ptr_->info(" -- [SUPER] PVA-TAIL: appended " + std::to_string(tail_len)
                                       + " m run-out past the goal");
                    }
                }
            }
        }

        /// WHTF-GUIDE-DEBUG (Phase 1 round 13, 2026-11-27): print the finished guide polyline when it
        /// changes enough, so the offline loop analysis can see whether the A* guide flips sides around
        /// a pillar cluster at the edge midpoints (the remaining S2 loops in 120a/121a/122a).
        if (std::getenv("WHTF_GUIDE_DEBUG") != nullptr && !guide_path.empty()) {
            static Vec3f last_front(9999, 9999, 9999);
            static double last_wall = -1e9;
            static size_t last_n = 0;
            double dev = 999.0;
            if (last_n == guide_path.size() && last_n > 0) {
                dev = 0.0;
                for (size_t i = 0; i < guide_path.size(); ++i) {
                    // store last guide in a function-static vector
                    static vec_Vec3f last_guide;
                    (void)last_guide;
                    break;
                }
                dev = (guide_path.front() - last_front).norm();
            }
            const double wall = ros_ptr_->getSimTime();
            if (wall - last_wall > 0.5 || dev > 0.5) {
                last_wall = wall;
                last_front = guide_path.front();
                last_n = guide_path.size();
                std::ostringstream os;
                os.setf(std::ios::fixed); os.precision(2);
                os << "GUIDEDBG wall=" << wall << " n=" << guide_path.size()
                   << " goal=(" << gi_.goal_p.x() << "," << gi_.goal_p.y() << ")"
                   << " len=" << guide_path_length << " astar=" << (astar_used ? 1 : 0) << " pts=";
                const size_t stride = std::max<size_t>(1, guide_path.size() / 40);
                for (size_t i = 0; i < guide_path.size(); i += stride) {
                    os << "(" << guide_path[i].x() << "," << guide_path[i].y() << ")";
                }
                os << " END=(" << guide_path.back().x() << "," << guide_path.back().y() << ")";
                std::cout << os.str() << std::endl;
            }
        }

        sfc.clear();
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizFrontendPath(guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }
        shifted_sfc_start_pt_ = Vec3f(9999,9999,9999);
        bool bool_ret_code = false;
        try {
            bool_ret_code = cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, shifted_sfc_start_pt_,
                                                          cfg_.use_fov_cut);
        } catch (const std::exception &e) {
            // 2026-11-28: a malformed guide (e.g. a via segment doubling back) threw out of the corridor
            // generator and KILLED fsm_node mid-flight, which the supervisor reported as "plan box silent".
            // A planning failure must be a RET_CODE, never a crash.
            ros_ptr_->warn(" -- [SUPER] corridor generation threw [{}] -- plan rejected", e.what());
            return FAILED;
        }

        if (!bool_ret_code) {
            ros_ptr_->warn(" -- [SUPER] SearchPolytopeOnPath for new path failed");
            return FAILED;
        }
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpSfc(sfc);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        time_consuming_[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();


        pos_fina_state.setZero();
        pos_fina_state.col(0) = guide_path.back();
        if (cfg_.goal_pva_en && connected_goal) {
            /// PVA GOAL (patch_goal_pva.py).  The terminal velocity comes from the GOAL, so a
            /// waypoint can be crossed without a stop while still being the goal of the
            /// trajectory.  Without this the corner is a rest point by construction.
            /// 2026-11-28 review P1-4: only apply it when the guide ACTUALLY connects to the goal;
            /// otherwise the optimiser is asked to hit the goal's exit velocity at a mid-leg point
            /// (REACH_HORIZON), which pulls the trajectory shape and can make it infeasible.
            pos_fina_state.col(1) = gi_.goal_v;
            pos_fina_state.col(2) = gi_.goal_a;
        } else if (cfg_.goal_pva_en) {
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [SUPER] PVA tail skipped: guide does not connect to the goal "
                               "(endpoint {:.2f} m away)", (pos_fina_state.col(0) - gi_.goal_p).norm());
            }
        } else if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
            pos_fina_state.col(1) = (gi_.goal_p - robot_state_.p).normalized() * cfg_.exp_traj_cfg.max_vel / 2;
        }
        if ((pos_fina_state.col(0) - gi_.goal_p).norm() < cfg_.resolution * 2) {
            if (!cfg_.goal_pva_en) { pos_fina_state.col(1).setZero(); }
            pos_fina_state.col(0) = gi_.goal_p;
        }

        // optimize and update exp traj
        bool temp_ret;
        Trajectory out_traj;
        TimeConsuming t_exp_opt("t_exp_opt", false);
        auto original_sfc = sfc;
        temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                           pos_fina_state,
                                           guide_path,
                                           guide_stamp,
                                           sfc,
                                           out_traj);
        if (!temp_ret && used_general_bc) {
            /// WHTF-GENERAL-BC RETRY (patch 17): a goal the current momentum cannot reach must not put
            /// the FSM into a failing loop -- retry ONCE from the historical zero-velocity start.
            ros_ptr_->warn(" -- [SUPER] GENERAL-BC: optimiser rejected the measured start state, "
                           "retrying from rest");
            pos_init_state.col(1).setZero();
            pos_init_state.col(2).setZero();
            sfc = original_sfc;
            temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                               pos_fina_state,
                                               guide_path,
                                               guide_stamp,
                                               sfc,
                                               out_traj);
            used_general_bc = false;
        }
        time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();
        {
            VecDf init_ts;
            vec_Vec3f init_ps;
            exp_traj_opt_->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        }
        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationExpTrajInPolytopes for new path failed");
            return FAILED;
        }
        /// WHTF-PLANDBG (round 15, temporary): does MINCO itself plan a low-speed dip near the start of a
        /// NEW-GOAL trajectory?  Print start velocity, minimum speed over the first 1.2 s, speed at 0.5 s
        /// and the trajectory duration, so the corner dip can be attributed to the plan or to the box.
        if (gi_.new_goal && !out_traj.empty()) {
            const double _dur = out_traj.getTotalDuration();
            const double _v0 = out_traj.getVel(0.0).norm();
            double _vmin = _v0;
            for (double _tt = 0.05; _tt <= std::min(1.2, _dur); _tt += 0.05) {
                _vmin = std::min(_vmin, out_traj.getVel(_tt).norm());
            }
            cout << " -- [SUPER] PLANDBG new_goal: dur=" << _dur << " v_start=" << _v0
                 << " v_min(1.2s)=" << _vmin
                 << " v(0.5s)=" << out_traj.getVel(std::min(0.5, _dur)).norm()
                 << " goal_dist=" << (gi_.goal_p - robot_state_.p).norm() << endl;
        }
        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);
        /// WHTF-REPLAN-TIME-LIMIT (patch 10): the same conflation as in ReplanOnce, with the same
        /// cost -- MEASURED 140 refusals in one sortie.  `replan_forward_dt` is a trajectory-time
        /// lookahead; the wall-clock budget is `replan_time_limit`.  See the long note there.
        if (replan_total_t > cfg_.replan_time_limit) {
            ros_ptr_->warn(" -- [SUPER] Replan over time({} s) > limit({} s)!!!! Return FAILED",
                           replan_total_t, cfg_.replan_time_limit);
            return FAILED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpTraj(out_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        double new_traj_WT = replan_process_start_WT;

        replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;
        Trajectory temp_exp_traj;
        if (!last_exp_traj_info_.empty() &&
            !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       temp_exp_traj)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.setSFC(sfc);
        temp_exp_traj = temp_exp_traj + out_traj;
        temp_exp_traj.start_WT = new_traj_WT; //last_exp_traj_info.replan_start_WT ;

        /// WHTF-TRAJ-CLEARANCE (2026-11-28): the Q2 analysis found the committed trajectory itself can
        /// pass within 3-7 cm of a trunk although the configured robot_r is 0.10 m -- CIRI/SFC does not
        /// enforce robot_r along the whole path (sparse map points / polytope slack).  Check the assembled
        /// trajectory against the planner's own INFLATED map (the same 0.2 m shell the A* uses).  Mode 1
        /// logs, mode 2 rejects the plan so the previous committed trajectory is kept.
        if (cfg_.traj_clearance_check_mode > 0) {
            const double _dur = temp_exp_traj.getTotalDuration();
            const double _dt = std::max(1e-3, cfg_.traj_clearance_dt);
            int _viol = 0;
            double _first_t = -1.0;
            Vec3f _first_p = Vec3f::Zero();
            for (double _tt = 0.0; _tt <= _dur + 1e-9; _tt += _dt) {
                Vec3f _p = temp_exp_traj.getPos(_tt);
                if (map_ptr_->isOccupiedInflate(_p)) {
                    if (_viol == 0) { _first_t = _tt; _first_p = _p; }
                    _viol++;
                }
            }
            if (_viol > 0) {
                static double _last_log_wt = -1e9;
                const double _now = ros_ptr_->getSimTime();
                if (_now - _last_log_wt > 1.0) {
                    _last_log_wt = _now;
                    ros_ptr_->warn(" -- [SUPER] TRAJ-CLEARANCE: {} sample(s) of the new committed trajectory "
                                   "are inside the INFLATED map (first t={:.2f} s, p=[{:.2f},{:.2f},{:.2f}], "
                                   "dur={:.2f} s, mode={})",
                                   _viol, _first_t, _first_p.x(), _first_p.y(), _first_p.z(), _dur,
                                   cfg_.traj_clearance_check_mode);
                }
                if (cfg_.traj_clearance_check_mode >= 2) {
                    return FAILED;
                }
            }
        }

        if (!last_exp_traj_info.empty()) {
            StatePVAJ yaw_replan_state;
            if (!guide_yaw_traj.getState(replan_state_TT, yaw_replan_state)) {
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            init_yaw = yaw_replan_state.row(0);
        }


        bool free_end{true};
        if (cfg_.goal_yaw_en && !isnan(gi_.goal_yaw) && connected_goal) {
            free_end = false;
            fina_yaw[0] = gi_.goal_yaw;
        }
        Trajectory new_traj, old_traj;

        if (!yaw_traj_opt_->optimize(init_yaw, fina_yaw, out_traj, new_traj, 3, false, free_end)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        if (!last_exp_traj_info.empty()) {
            if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                           old_traj)) {
                ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
        }

        const auto temp_yaw_traj = old_traj + new_traj;
        // check if part of the exp on last backup
        double on_backup_end_TT{-1}, on_backup_start_TT{-1};
        if (!last_exp_traj_info.empty() && replan_state_TT > cmd_traj_info_.getBackupTrajStartTT()) {
            on_backup_start_TT = cmd_traj_info_.getBackupTrajStartTT() - replan_process_start_TT;
            on_backup_end_TT = replan_state_TT - replan_process_start_TT;
        }
        out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj, on_backup_start_TT,
                                        on_backup_end_TT);

        latest_replan.setExpYawTraj(temp_yaw_traj);
        latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

    RET_CODE SuperPlanner::generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info) {
        drone_state_mutex_.lock();
        back_traj_info.setRobotPos(robot_state_.p);
        drone_state_mutex_.unlock();
        TimeConsuming t_back_frontend("t_back_frontend", false);
        double total_dur = ref_exp_traj.getTotalDuration();
        double start_t = ros_ptr_->getSimTime() - ref_exp_traj.getStartWallTime();


        if (start_t > total_dur - 0.01) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [generateBackupTrajectory]: start_t > total_dur, return NO_NEED");
            }
            return NO_NEED;
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        // 同时记录每一个点的刹车时间和刹车距离
        vector<double> min_stop_dis;
        vector<TimePosPair> eval_ps;
        Vec3f temp_vel;

        // 记录当前时刻到最远时刻的所有可视部分
        Vec3f last_pos = ref_exp_traj.getPos(start_t);
        for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt) {
            temp_point = ref_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) {
                continue;
            }
            last_pos = temp_point;
            temp_vel = ref_exp_traj.getVel(out_t);
            // Compute initial
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.push_back(std::pair<double, Vec3f>(out_t, temp_point));
            const double min_dis =
                    cfg_.sensing_horizon > 0 ? std::min(cfg_.sensing_horizon, cfg_.safe_corridor_line_max_length)
                                             : cfg_.safe_corridor_line_max_length;
            if (!map_ptr_->isLineFree(back_traj_info.getRobotPos(),
                                      temp_point,
                                      min_dis,
                                      cfg_.seed_line_neighbour)) {
                all_traj_visible = false;
                break;
            }
        }

        if (all_traj_visible) {
            back_traj_info.setEmpty();
            {
                double dur = ref_exp_traj.getTotalDuration();
                Vec3f seed_pt = ref_exp_traj.getPos(dur);
                Line line{back_traj_info.getRobotPos(), seed_pt};
                Polytope temp_poly;
                if (cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
                    back_traj_info.setSFC(temp_poly);
                    {
                        TimeConsuming t_viz("tviz", false);
                        ros_ptr_->vizBackupSfc(temp_poly);
                        time_consuming_[VISUALIZATION] += t_viz.stop();
                    }
                }
            }
            return FINISH;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= cfg_.sample_traj_dt;
            Vec3f out_p = ref_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > cfg_.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);

        // TODO check this logic, comment on Dec. 13
        // if
        // 1) last exp traj has a backup traj
        // 2) last backup WT is larger than this term
        // 3) last exp is collision free
        // if (ref_exp_traj.back_traj_start_TT > 0 &&
        // seed_point_t < ref_exp_traj.back_traj_start_TT) {
        // return NO_NEED;
        // }


        Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);

        Vec3f shifted_robot_p = shifted_sfc_start_pt_.norm()> 999?robot_state_.p:shifted_sfc_start_pt_;
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, shifted_robot_p, shifted_robot_p, 3.0)) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }

        Line line{shifted_robot_p, seed_point};
        Polytope temp_poly;
        if (!cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        Eigen::Matrix3Xd vPoly;
        if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
            ros_ptr_->warn(" -- [SUPER] Cannot generate feasible backup sfc, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        if (cfg_.use_fov_cut) {
            if (!fov_checker_->cutPolyByFov(robot_state_.p, robot_state_.q, seed_point,
                                            temp_poly)) {
                ros_ptr_->warn(" -- [SUPER] cutPolyByFov failed, force return");
                return FAILED;
            }
        }
        // cut by sensing horizon
        if (cfg_.sensing_horizon > 0 &&
            !fov_checker_->cutPolyBySensingHorizon(robot_state_.p, seed_point, cfg_.sensing_horizon,
                                                   temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] cutPolyBySensingHorizon failed, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        back_traj_info.setSFC(temp_poly);

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizBackupSfc(temp_poly);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

//        Vec3f out_p = temp_point;
//        double t_R = 0.0;
        double eval_t = eval_ps.back().first + cfg_.sample_traj_dt;
        last_pos = eval_ps.back().second;
        while (temp_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = ref_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
                eval_t += cfg_.sample_traj_dt;
                continue;
            }
            temp_vel = ref_exp_traj.getVel(out_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt;
        }
        eval_ps.pop_back();
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        //        bool use_new{true};
        //        if (use_new) {
        double t0 = ros_ptr_->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        //            cout << "t0: " << t0 << endl;
        //            cout << "te: " << te << endl;
        //            cout << "exp_traj_dur: " << ref_exp_traj.optimized_exp_traj.getTotalDuration() << endl;
        double vel_e_n = ref_exp_traj.getVel(te).norm();
        double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
        double heu_dur = te - heu_ts;
        Vec3f heu_p = seed_point;
        time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        Trajectory temp_pos_traj;
        auto sfc0 = back_traj_info.getSFC();
        bool temp_ret = back_traj_opt_->optimize(ref_exp_traj.posTraj(),
                                                 t0,
                                                 te,
                                                 heu_ts,
                                                 heu_p,
                                                 heu_dur,
                                                 back_traj_info.getSFC(),
                                                 temp_pos_traj,
                                                 opt_ts);
        time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();

        {
            double init_ts;
            VecDf init_times;
            vec_Vec3f init_ps;
            back_traj_opt_->getInitValue(init_ts, init_times, init_ps);
            latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                             t0, te,
                                             back_traj_info.getSFC());
            Trajectory traj;
            double out_ts;
            back_traj_opt_->optimize(ref_exp_traj.posTraj(),
                                     t0,
                                     te,
                                     init_ts,
                                     sfc0,
                                     init_times,
                                     init_ps,
                                     traj,
                                     out_ts
            );

        }

        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationBakTrajInPolytopes failed, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        } else {
            Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
            Vec4f yaw_goal{0, 0, 0, 0};
            bool free_end{true};
            if (cfg_.goal_yaw_en) {
                if (!isnan(gi_.goal_yaw)) {
                    free_end = false;
                    yaw_goal[0] = gi_.goal_yaw;
                }
            }
            Trajectory temp_yaw_traj;
            if (!yaw_traj_opt_->optimize(yaw_init_vec, yaw_goal, temp_pos_traj,
                                         temp_yaw_traj, 3, false, free_end)) {
                ros_ptr_->error(" -- [SUPER] in [generateBackupTrajectory] YawTrajOpt FAILD.");
                return OPT_FAILED;
            }


            if (opt_ts < t0) {
                ros_ptr_->error(" -- [SUPER] opt_ts {} < t0 {}", opt_ts, t0);
                return OPT_FAILED;
            }
            double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
            const auto &committed_ts_WT = cmd_traj_info_.getBackupTrajStartTT();
            if (committed_ts_WT < cmd_traj_info_.getTotalDuration() && new_ts_WT < committed_ts_WT) {
                ros_ptr_->error(" -- [SUPER] new_ts_WT {} < committed_ts_WT {}", new_ts_WT, committed_ts_WT);
                return OPT_FAILED;
            }


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizBackupTraj(temp_pos_traj);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
            latest_replan.setBackupTraj(temp_pos_traj);
            latest_replan.setBackupYawTraj(temp_yaw_traj);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] Cannot find backup traj start point.");
        return FAILED;
    }

    int SuperPlanner::getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt) {
        if (goals.size() == 1) {
            return 0;
        }
        Vec3f a = start_pt, b;
        int min_id = 0;
        double min_dis = 1e10;
        for (long unsigned int i = 0; i < goals.size() - 1; i++) {
            b = goals[i];
            double dis = geometry_utils::pointLineSegmentDistance(start_pt, a, b);
            if (dis < min_dis) {
                min_dis = dis;
                min_id = i;
            }
            a = b;
        }
        return min_id;
    }

    bool
    SuperPlanner::PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                             const double &searching_horizon,
                             vec_Vec3f &path) {
        using namespace path_search;
        if (searching_horizon <= 0.0) {
            ros_ptr_->error(" -- [SUPER] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        rog_map::GridType start_type;
        start_type = map_ptr_->getGridType(start_pt);
        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (start_type == rog_map::GridType::OCCUPIED ||
            start_type == rog_map::GridType::OUT_OF_MAP) {
            ros_ptr_->warn(
                    " -- [SUPER] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
        vec_Vec3f out_path;
        RET_CODE ret_es = astar_ptr_->escapePathSearch(start_pt, flag_es, out_path);
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                ros_ptr_->error(
                        " -- [SUPER] Escape path search failed with [{}], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
        //            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) | DONT_USE_INF_NEIGHBOR;

        RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                               path);

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        if (ret_code == NO_PATH) {
            flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                          path);
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        }
        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            ros_ptr_->error(
                    " -- [SUPER] Path search failed with [{}], force return.\n", RET_CODE_STR[ret_code].c_str());
            return false;
        }
        if (!start_point_escape_path.empty()) {
            path.insert(path.begin(), start_point_escape_path.begin(),
                        start_point_escape_path.end());
        }

        if (path.empty()) {
            ros_ptr_->warn(
                    " -- [SUPER] Path search failed with empty segments, force return.");
            return false;
        }
        path.insert(path.begin(), start_pt);
        if (ret_code == REACH_GOAL) {
            path.push_back(goal);
        }
        return true;
    }


    void SuperPlanner::getRobotState(rog_map::RobotState &out) {
        robot_state_ = map_ptr_->getRobotState();
        out = robot_state_;
    }
}
