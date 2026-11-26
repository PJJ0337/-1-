# 회귀 테스트 보고서 (자동 생성 — tools/rebuild_and_test.py)

- 일시 2026-11-26 22:12 · params `config/params.yaml` (updated 2026-11-25) · shoulder_case **a2_doc** · payload_limit False · reach_limit False · 짐 3.0 kg
- 재빌드: build_v1.py (B1_A2_CASE=a2_doc) 1.2 s · b1.xml md5 `b4a314ad` — 저장소 원본 `b4a314ad` 와 **같음**
- pytest (전체): **26/26 통과** · 59 s
- 통과 기준 · 허용 오차의 이유는 각 테스트 파일 머리 주석(tests/test_*.py)과 config/params.yaml regression 에 있음

| 파일 | 테스트 | 결과 | 시간 s | 기록 숫자 |
|---|---|---|---|---|
| test_ik | test_1_thirteen_targets | ✅ | 1.2 | wb_ok 10/13 |
| test_ik | test_2_3_random_convergence | ✅ | 6.2 | pos_rate 100.0 % · down_rate 78.0 % |
| test_ik | test_4_reach_definition_matches_taehoon | ✅ | 0.4 | max_diff_m 1.1e-16 |
| test_ik | test_5_reach_limit_noop_and_binding | ✅ | 0.1 | A3_reach_free_m 0.449 · A3_reach_lim_m 0.301 · A3_base_move_free_lim_m 0.203 → 0.695 |
| test_ik | test_5b_unbound_bit_identical | ✅ | 0.0 |  |
| test_ik | test_6_load_reach_limit | ✅ | 0.0 |  |
| test_scenario | test_1_2_pick20 | ✅ | 38.3 | success 19/20 · fails {1: 'ik_fail'} · goal_err_mm 6.0 / 13.6 · t_cycle_s 24.5 |
| test_scenario | test_3_flags_noop[0] | ✅ | 4.1 | t_total 24.3 / 24.3 / 24.3 |
| test_scenario | test_3_flags_noop[3] | ✅ | 4.2 | t_total 24.5 / 24.5 / 24.5 |
| test_scenario | test_4_reach_binding | ✅ | 2.7 | seed13_reach_carry_free_lim 0.378 → 0.366 · success_lim True |
| test_scenario | test_5_payload_flag | ✅ | 0.0 |  |
| test_torque | test_margin_definition | ✅ | 0.0 |  |
| test_torque | test_need_regression | ✅ | 0.0 |  |
| test_torque | test_both_cases_pass_at_target[a2_doc] | ✅ | 0.0 |  |
| test_torque | test_both_cases_pass_at_target[cad_ri85] | ✅ | 0.0 |  |
| test_torque | test_configured_case_with_flags | ✅ | 0.1 |  |
| test_torque | test_mmax_regression_and_linear[TQ-a2_doc] | ✅ | 0.1 |  |
| test_torque | test_mmax_regression_and_linear[RI85-cad_ri85] | ✅ | 0.1 |  |
| test_torque | test_payload_limit_flag | ✅ | 0.1 |  |
| test_torque | test_reach_limit_flag | ✅ | 0.0 |  |
| test_torque | test_hand_check_slope | ✅ | 0.0 |  |
| test_validation5 | test_1_mass | ✅ | 0.0 |  |
| test_validation5 | test_2_hold_forward | ✅ | 0.1 |  |
| test_validation5 | test_3_gravity_torque | ✅ | 0.5 |  |
| test_validation5 | test_4_range_sweep_no_self_collision | ✅ | 0.1 |  |
| test_validation5 | test_5_mirror | ✅ | 0.0 |  |

담당: test_validation5 · test_torque = 함태훈(df1e64d 사본) / test_ik · test_scenario = 박종진
