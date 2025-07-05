// parameter_descriptions.dart – centralized SLAM parameter help text
// 같은 디렉터리에 두고 SlamDashboard에서 import 하세요.

/// 각 SLAM 파라미터에 대한 간단한 설명을 담은 상수 Map.
/// 실제 프로젝트 상황에 맞추어 자유롭게 보강/수정하면 됩니다.
const Map<String, String> parameterDescriptions = {
  // ───── scan_matcher ─────
  'ndt_resolution':
      'NDT 스캔 매칭에 사용되는 보셀(voxel) 분해능 (m). 값이 작을수록 정밀하지만 계산량이 증가합니다.',
  'ndt_num_threads':
      'NDT 계산에 사용할 쓰레드 개수. CPU 코어 수를 넘지 않는 범위에서 증가시키면 속도가 향상됩니다.',
  'gicp_corr_dist_threshold':
      'GICP 상관 거리 한계값(m). 스캔 포인트 간 최대 대응 거리입니다.',
  'trans_for_mapupdate':
      '지도 업데이트를 수행하기 위한 최소 변위 (m).',
  'vg_size_for_input':
      '입력 스캔에 대한 다운샘플링 보셀 크기 (m).',
  'vg_size_for_map':
      '지도 포인트클라우드 다운샘플링 보셀 크기 (m).',
  'use_min_max_filter':
      '최소/최대 거리 필터 사용 여부. true이면 지정 거리 범위를 벗어난 포인트를 제거합니다.',
  'scan_min_range': '스캔의 최소 유효 거리 (m).',
  'scan_max_range': '스캔의 최대 유효 거리 (m).',
  'scan_period':
      '스캔 주기(초). 센서의 회전 속도나 프레임 레이트와 연계하여 조정합니다.',

  // ───── set_initial_pose ─────
  'set_initial_pose':
      '초기 위치를 수동으로 설정할지 여부. true이면 아래 초기값(qx~qw 포함)을 사용합니다.',
  'initial_pose_x': '초기 위치 X 좌표 (m).',
  'initial_pose_y': '초기 위치 Y 좌표 (m).',
  'initial_pose_z': '초기 위치 Z 좌표 (m).',
  'initial_pose_qx': '초기 자세 quaternion X.',
  'initial_pose_qy': '초기 자세 quaternion Y.',
  'initial_pose_qz': '초기 자세 quaternion Z.',
  'initial_pose_qw': '초기 자세 quaternion W.',

  // ───── graph_based_slam ─────
  'gb_ndt_resolution':
      '그래프 기반 SLAM 단계에서 사용되는 NDT 보셀 분해능 (m).',
  'gb_ndt_num_threads':
      '그래프 기반 SLAM용 NDT 계산 쓰레드 수.',
  'voxel_leaf_size':
      '그래프 기반 SLAM 입력 포인트 다운샘플링 보셀 크기 (m).',
  'loop_detection_period':
      '루프 클로저 탐지 주기 (ms). 값이 작을수록 자주 탐색하지만 CPU 부담이 커집니다.',
  'threshold_loop_closure_score':
      '루프 클로저 유효성 판단 점수 임계값. 낮을수록 민감하게 루프를 검출합니다.',
  'distance_loop_closure':
      '루프 클로저 후보 간 최대 거리 (m).',
  'range_of_searching_loop_closure':
      '루프 클로저 탐색 범위 (m).',
  'search_submap_num':
      '루프 클로저 검색 시 사용할 서브맵 개수.',
  'num_adjacent_pose_constraints':
      '그래프 최적화 시 인접 포즈 간 추가 제약 개수.',
  'use_save_map_in_loop': '루프 클로저 중간에 지도를 저장할지 여부.',
  'debug_flag': '디버그 메시지 출력 여부.',
};
