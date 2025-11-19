%% MAIN PROGRAM
% Actually a wrapper to the optimization scripts...

if opt_strategy == "centralized"
    cbf_centralized;
elseif opt_strategy == "decentralized"
    cbf_decentralized;
elseif opt_strategy == "hybrid"
    cbf_hybrid;
else
    disp("ERROR: no optimization approach has been defined. " + ...
        "Provide a reasonable value to 'opt_strategy' parameter")
    return;
end