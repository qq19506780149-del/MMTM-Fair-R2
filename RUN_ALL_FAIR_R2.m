function RUN_ALL_FAIR_R2()
% Execute the six map-family experiments under the frozen FAIR_R2 protocol.
% Place all FAIR_R2_*.m files in the current MATLAB working folder.
FAIR_R2_MAPA_SSA();
FAIR_R2_MAPA_NGO();
FAIR_R2_MAPA_GWO();
FAIR_R2_MAPB_SSA();
FAIR_R2_MAPB_NGO();
FAIR_R2_MAPB_GWO();
fprintf('FAIR_R2: all six experiments completed. Run verify_map_ab_outputs.py on the output directory.\n');
end