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
  message?: string;
  error?: string;
  status?: string;
  rd_matrix_shape?: [number, number];
  num_detections?: number;
  tracker_estimate?: number[];
}

// Propulsion Specific Types
export type PropulsionData = Record<string, unknown>;

export type SimulationPayload = Record<string, any>;
