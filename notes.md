Nice. Where I'd go next, in order of leverage:                                                               
                                                                                      
  1. Make cam1 actually autonomous on reboot — add irlink daemon-listen --pixel <coords> to /etc/rc.local on   
  cam1, where <coords> come from /opt/etc/calibration.json. One-line shell change + a tiny coord-extraction
  one-liner. Without this, cam1 still needs SSH to start every time — every test we've run is dev-mode-only.   
  2. Cold-reboot validation — ssh dacam1 reboot, wait 60s, don't touch cam1, run bicall from the laptop. If
   everything we built actually works, this round just succeeds. This is the deployment-ready milestone; it's  
  also where any persistence/timing bugs surface.
  3. Wire session.py to consume PIXEL: and PEER-CAL: — replaces cal_procedure.py for in-band re-cals and lets  
  you orchestrate from Python rather than piping bicall via SSH. Mechanical, ~30 lines.                        
  4. Pre-flight health check — your earlier "feedback that we're in range" ask. Could be as simple as connect →
   ping → STATS → recommend bicall? y/n. Easy if you want it; not load-bearing for autonomy.                   
  5. Cold bootstrap (no /opt/etc/calibration.json at all) — real research. Stage 2 showed grid mode at 200 ms
  is below decode threshold at this distance. Likely needs even slower rates, multi-block parallel decode, or  
  both. Skippable if persistent calibration is good enough — the first cal is done in dev anyway, and JFFS2
  survives reboots.    
