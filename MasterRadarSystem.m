classdef MasterRadarSystem < handle
    % MASTERRADARSYSTEM - Complete Mathematical Physics Engine
    
    properties (Constant)
        c = 299792458;          
        k_B = 1.380649e-23;     
        T0 = 290;               
    end
    
    properties (SetAccess = public)
        ArrayConfig
        HardwareSpecs
        TargetState
        EnvironmentState
    end
    
    methods
        function obj = MasterRadarSystem()
            % Initialize with baseline parameters
            obj.ArrayConfig = struct('Tx_Ny', 8, 'Tx_Nz', 8, 'Rx_Ny', 8, 'Rx_Nz', 8, 'd_lambda', 0.5, 'Taper', 1);
            obj.HardwareSpecs = struct('Freq', 3e9, 'Pt', 100e3, 'Loss_dB', 5, 'NF_dB', 3, 'PRF', 2e3, 'Tau', 10e-6, 'BW', 5e6);
            obj.TargetState = struct('RCS', 5, 'Range', 50e3, 'Az', 20, 'El', 10, 'Vel', -300, 'Heading', 180);
            obj.EnvironmentState = struct('ScanAz', 20, 'ScanEl', 10, 'JammerAz', -30, 'JammerEl', 5, 'JSR_dB', 30, 'UseMVDR', false);
        end
        
        function [metrics] = calcRadarFundamentals(obj)
            lambda = obj.c / obj.HardwareSpecs.Freq;
            G_elem = pi; 
            G_tx = G_elem * (obj.ArrayConfig.Tx_Ny * obj.ArrayConfig.Tx_Nz);
            G_rx = G_elem * (obj.ArrayConfig.Rx_Ny * obj.ArrayConfig.Rx_Nz);
            
            L_lin = 10^(obj.HardwareSpecs.Loss_dB / 10);
            F_lin = 10^(obj.HardwareSpecs.NF_dB / 10);
            
            Pr = (obj.HardwareSpecs.Pt * G_tx * G_rx * lambda^2 * obj.TargetState.RCS) / ...
                 (((4*pi)^3) * obj.TargetState.Range^4 * L_lin);
            Pn = obj.k_B * obj.T0 * obj.HardwareSpecs.BW * F_lin;
            
            metrics.SNR_dB = 10 * log10(Pr / Pn);
            metrics.R_res_raw = (obj.c * obj.HardwareSpecs.Tau) / 2;
            metrics.R_res_comp = obj.c / (2 * obj.HardwareSpecs.BW);
            metrics.R_max = ((obj.HardwareSpecs.Pt * G_tx * G_rx * lambda^2 * obj.TargetState.RCS) / ...
                            (((4*pi)^3) * Pn * L_lin * 10^(13/10)))^0.25;
            metrics.R_unambig = obj.c / (2 * obj.HardwareSpecs.PRF);
            metrics.DutyCycle = obj.HardwareSpecs.Tau * obj.HardwareSpecs.PRF;
            metrics.AvgPower = obj.HardwareSpecs.Pt * metrics.DutyCycle;
        end
        
        function [AZ, EL, Pat_dB, az_cut, el_cut] = calcArrayPattern(obj)
            % Strict Discrete Summation for perfect null-steering accuracy
            lambda = obj.c / obj.HardwareSpecs.Freq;
            d = obj.ArrayConfig.d_lambda * lambda;
            
            az_vec = linspace(-90, 90, 90); % 90x90 grid for UI responsiveness
            el_vec = linspace(-90, 90, 90);
            [AZ, EL] = meshgrid(az_vec, el_vec);
            
            NumRx = obj.ArrayConfig.Rx_Ny * obj.ArrayConfig.Rx_Nz;
            
            calcSteeringVec = @(az, el, Ny, Nz) exp(1j * (2*pi/lambda) * ...
                (kron(ones(Nz,1), ((0:Ny-1)*d - mean((0:Ny-1)*d))') * cosd(el)*sind(az) + ...
                 kron(((0:Nz-1)*d - mean((0:Nz-1)*d))', ones(Ny,1)) * sind(el)));

            v_scan = calcSteeringVec(obj.EnvironmentState.ScanAz, obj.EnvironmentState.ScanEl, obj.ArrayConfig.Rx_Ny, obj.ArrayConfig.Rx_Nz);
            
            % Implement MVDR or Uniform/Hamming Taper
            if obj.EnvironmentState.UseMVDR
                v_jam = calcSteeringVec(obj.EnvironmentState.JammerAz, obj.EnvironmentState.JammerEl, obj.ArrayConfig.Rx_Ny, obj.ArrayConfig.Rx_Nz);
                Pn = obj.k_B * obj.T0 * obj.HardwareSpecs.BW * 10^(obj.HardwareSpecs.NF_dB/10);
                Pr_targ = (obj.HardwareSpecs.Pt * pi*(obj.ArrayConfig.Tx_Ny*obj.ArrayConfig.Tx_Nz) * pi*NumRx * lambda^2 * obj.TargetState.RCS) / ...
                          (((4*pi)^3) * obj.TargetState.Range^4 * 10^(obj.HardwareSpecs.Loss_dB/10));
                P_jam = Pr_targ * 10^(obj.EnvironmentState.JSR_dB/10);
                
                R_in = Pn * eye(NumRx) + P_jam * (v_jam * v_jam');
                R_inv_v = R_in \ v_scan;
                w_rx = R_inv_v / (v_scan' * R_inv_v);
            else
                w_y = ones(obj.ArrayConfig.Rx_Ny, 1);
                w_z = ones(obj.ArrayConfig.Rx_Nz, 1);
                if obj.ArrayConfig.Taper == 2 % Hamming
                    w_y = hamming(obj.ArrayConfig.Rx_Ny);
                    w_z = hamming(obj.ArrayConfig.Rx_Nz);
                end
                W_2D = w_z * w_y';
                w_rx = W_2D(:) .* v_scan;
                w_rx = w_rx / norm(w_rx);
            end

            % Calculate 2D Pattern Map
            Pat_linear = zeros(size(AZ));
            for i = 1:numel(AZ)
                v_theta = calcSteeringVec(AZ(i), EL(i), obj.ArrayConfig.Rx_Ny, obj.ArrayConfig.Rx_Nz);
                Pat_linear(i) = abs(w_rx' * v_theta)^2;
            end
            
            Pat_dB = 10 * log10(Pat_linear / max(Pat_linear(:)) + 1e-12);
            Pat_dB = max(Pat_dB, -50); 
            
            [~, el_idx] = min(abs(el_vec - obj.EnvironmentState.ScanEl));
            [~, az_idx] = min(abs(az_vec - obj.EnvironmentState.ScanAz));
            az_cut = Pat_dB(el_idx, :);
            el_cut = Pat_dB(:, az_idx)';
        end
        
        function [t_rx, rx_raw, mf_out_dB, r_axis, RD_Map, v_axis] = processCPI(obj)
            % Complete CPI Processing Pipeline (Time, MF, and RD Map)
            N_pulses = 16;
            Fs = max(2.5 * obj.HardwareSpecs.BW, 10e6);
            t_tx = -obj.HardwareSpecs.Tau/2 : 1/Fs : obj.HardwareSpecs.Tau/2;
            K = obj.HardwareSpecs.BW / obj.HardwareSpecs.Tau; 
            s_tx = exp(1j * pi * K * t_tx.^2);
            mf_ref = conj(fliplr(s_tx)) .* hamming(length(s_tx))';
            
            PRI = 1 / obj.HardwareSpecs.PRF;
            T_sim_sig = min(PRI, (obj.TargetState.Range * 1.5 * 2)/obj.c);
            t_rx = 0 : 1/Fs : T_sim_sig;
            
            lambda = obj.c / obj.HardwareSpecs.Freq;
            v_radial = obj.TargetState.Vel * cosd(obj.TargetState.Heading - obj.TargetState.Az);
            
            % Calculate Powers
            L_lin = 10^(obj.HardwareSpecs.Loss_dB/10);
            F_lin = 10^(obj.HardwareSpecs.NF_dB/10);
            G_tx = pi * obj.ArrayConfig.Tx_Ny * obj.ArrayConfig.Tx_Nz;
            G_rx = pi * obj.ArrayConfig.Rx_Ny * obj.ArrayConfig.Rx_Nz;
            Pr_targ = (obj.HardwareSpecs.Pt * G_tx * G_rx * lambda^2 * obj.TargetState.RCS) / (((4*pi)^3) * obj.TargetState.Range^4 * L_lin);
            Pn = obj.k_B * obj.T0 * obj.HardwareSpecs.BW * F_lin;
            
            % Jammer setup
            P_jam = 0;
            if obj.EnvironmentState.UseMVDR % Simulating JSR environment if MVDR checked or unchecked
                P_jam = Pr_targ * 10^(obj.EnvironmentState.JSR_dB/10);
            end
            
            rx_matrix = zeros(N_pulses, length(t_rx));
            for p = 1:N_pulses
                current_R = obj.TargetState.Range - (v_radial * (p-1) * PRI);
                idx_start = round(((2 * current_R) / obj.c) * Fs);
                idx_end = idx_start + length(s_tx) - 1;
                
                sig = zeros(1, length(t_rx));
                if idx_end <= length(t_rx)
                    phase_shift = exp(-1j * 4 * pi * current_R / lambda);
                    sig(idx_start:idx_end) = sqrt(Pr_targ) * s_tx * phase_shift; 
                end
                
                % Add noise and jammer
                noise = sqrt(Pn) * (randn(1, length(t_rx)) + 1j*randn(1, length(t_rx))) / sqrt(2);
                jammer = sqrt(P_jam) * (randn(1, length(t_rx)) + 1j*randn(1, length(t_rx))) / sqrt(2);
                
                rx_matrix(p, :) = sig + noise + jammer;
            end
            
            rx_raw = real(rx_matrix(1,:));
            
            mf_matrix = zeros(size(rx_matrix));
            for p = 1:N_pulses
                mf_matrix(p,:) = conv(rx_matrix(p,:), mf_ref, 'same'); 
            end
            
            r_axis = (t_rx * obj.c) / 2;
            mf_out_dB = 20*log10(abs(mf_matrix(1,:)) + 1e-12);
            mf_out_dB = mf_out_dB - max(mf_out_dB);
            
            RD_Map = fftshift(fft(mf_matrix, [], 1), 1);
            RD_Map = 20*log10(abs(RD_Map) + 1e-12);
            RD_Map = RD_Map - max(RD_Map(:)); 
            
            doppler_axis = linspace(-obj.HardwareSpecs.PRF/2, obj.HardwareSpecs.PRF/2, N_pulses);
            v_axis = doppler_axis * lambda / 2;
        end
        
        function [opt_Pt, opt_NyNz, logStr] = runOptimizer(obj, reqSNR, maxPt)
            % System Design Optimizer Logic
            lambda = obj.c / obj.HardwareSpecs.Freq;
            L_lin = 10^(obj.HardwareSpecs.Loss_dB/10);
            F_lin = 10^(obj.HardwareSpecs.NF_dB/10);
            req_snr_lin = 10^(reqSNR/10);
            
            Pn = obj.k_B * obj.T0 * obj.HardwareSpecs.BW * F_lin;
            
            logStr = sprintf('--- Optimization Results ---\nTarget Range: %.1f km\nRequired SNR: %.1f dB\n', obj.TargetState.Range/1000, reqSNR);
            
            % Assume Square Array Tx=Rx
            opt_Pt = maxPt * 1000;
            opt_NyNz = 8;
            success = false;
            
            for N = 4:2:64
                G_total = pi * (N*N);
                Pt_req = (req_snr_lin * (((4*pi)^3) * obj.TargetState.Range^4 * Pn * L_lin)) / ...
                         (G_total * G_total * lambda^2 * obj.TargetState.RCS);
                
                if Pt_req <= (maxPt * 1000)
                    opt_Pt = Pt_req;
                    opt_NyNz = N;
                    success = true;
                    break;
                end
            end
            
            if success
                logStr = [logStr, sprintf('SUCCESS:\nRequired Elements: %d x %d (Tx & Rx)\nRequired Power: %.2f kW\n', opt_NyNz, opt_NyNz, opt_Pt/1000)];
            else
                logStr = [logStr, sprintf('FAILED:\nMax power reached (%.1f kW). Try increasing allowed power or lowering required SNR.\n', maxPt)];
            end
        end
    end
end