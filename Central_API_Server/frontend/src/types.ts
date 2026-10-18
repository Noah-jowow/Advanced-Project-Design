// Removed unused import
export interface WsLogMessage {
  type: 'log';
  message: string;
}

export interface WsResultMessage<T = Record<string, unknown>> {
  type: 'result';
  data: T;
}

export type WsMessage = WsLogMessage | WsResultMessage;

// CFD Specific Types
export interface CFDData {
  mode?: string;
  error?: string;
  
  // Geometry/Domain
  x?: number[];
  y?: number[];
  z?: number[];
  i?: number[];
  j?: number[];
  k?: number[];
  c?: number[]; // boundary tags
  
  aero_x?: number[];
  aero_y?: number[];
  aero_z?: number[];
  aero_i?: number[];
  aero_j?: number[];
  aero_k?: number[];

  // Post Processing
  post_x?: number[];
  post_y?: number[];
  post_z?: number[];
  post_i?: number[];
  post_j?: number[];
  post_k?: number[];
  post_mach?: number[];
  post_velocity?: number[];
  post_pressure?: number[];
  
  // Solver results
  cfd_x?: number[];
  cfd_y?: number[];
  cfd_z?: number[];
  cfd_i?: number[];
  cfd_j?: number[];
  cfd_k?: number[];
  cfd_mach?: number[];
  
  cl?: number[];
  cd?: number[];
  residual?: number[];
  res_rho?: number[];
  res_rhoU?: number[];
  res_rhoE?: number[];
}

// Radar Specific Types
export interface RadarData {
  rd_matrix?: number[][];
  stap_weights?: string[];
  stap_covariance_norm?: number;
  stap_covariance?: number[][];
  message?: string;
  error?: string;
  status?: string;
  rd_matrix_shape?: [number, number];
  num_detections?: number;
  tracker_estimate?: number[];
  
  // Spatial pattern
  spatial_pattern?: number[][];
  azimuth_cut?: number[];
  elevation_cut?: number[];

  // 1D and CFAR
  cfar_detections?: number[][];
  mf_profile?: number[];
  snr_range?: number[];

  // Tracker History and Ellipsoid
  track_history_x?: number[];
  track_history_y?: number[];
  track_history_z?: number[];
  ellipsoid_x?: number[];
  ellipsoid_y?: number[];
  ellipsoid_z?: number[];

  // Pointer Dynamics
  hex_tp?: number[][];
  hex_base?: number[][];
  hex_laser?: number[];
  hex_forces?: number[];
  hex_warn?: string;
  hex_cond?: number;
  azel_q?: number[];
  azel_tau_a?: number;
  azel_tau_e?: number;
  azel_warn?: string;

  // C2 Analytics & Errors
  c2_measurements?: {
    id: number;
    snr_db: number;
    pol: number[];
  }[];
  filter_range_error?: number[];
  filter_velocity_error?: number[];

  // Baseband & EW
  fast_time_rx?: number[];
  fast_time_t_us?: number[];
  ew_freq_mhz?: number[];
  ew_power_dbm?: number[];
  ppi_blips?: { id: number; r: number; theta: number }[];
}

// SADC (Spacecraft Attitude Dynamics & Control) Specific Types
export interface SADCData {
  status?: string;
  error?: string;
  time?: number[];
  start_time?: number;
  target_mode?: string;
  target_idx?: number;
  target_name?: string;

  // Euler angles [deg] — Roll, Pitch, Yaw
  euler_actual_r?: number[];
  euler_actual_p?: number[];
  euler_actual_y?: number[];
  euler_cmd_r?: number[];
  euler_cmd_p?: number[];
  euler_cmd_y?: number[];
  euler_error_r?: number[];
  euler_error_p?: number[];
  euler_error_y?: number[];

  // Angular velocity [deg/s]
  omega_x?: number[];
  omega_y?: number[];
  omega_z?: number[];

  // Orbital state [km, km/s]
  orbit_x?: number[];
  orbit_y?: number[];
  orbit_z?: number[];
  vel_x?: number[];
  vel_y?: number[];
  vel_z?: number[];
  altitude?: number[];

  // Quaternions [q1, q2, q3, q4] (scalar-last)
  q1?: number[];
  q2?: number[];
  q3?: number[];
  q4?: number[];
  qt1?: number[];
  qt2?: number[];
  qt3?: number[];
  qt4?: number[];
  qe1?: number[];
  qe2?: number[];
  qe3?: number[];
  qe4?: number[];

  // Torques [N·m]
  torque_ctrl_x?: number[];
  torque_ctrl_y?: number[];
  torque_ctrl_z?: number[];
  torque_gg_x?: number[];
  torque_gg_y?: number[];
  torque_gg_z?: number[];

  // Spacecraft visualization (final state)
  sc_dcm?: number[][];
  sc_pos?: number[];

  // Ground station overpass telemetry
  gs_elevation?: number[];
  gs_azimuth?: number[];
  gs_range?: number[];
  gs_lat?: number[];
  gs_long?: number[];
  gs_eci?: number[][];
  gs_names?: string[];
}


// Propulsion Specific Types
export type PropulsionData = Record<string, unknown>;

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type SimulationPayload = Record<string, any>;

