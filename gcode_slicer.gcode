; --- Inicio troca filamento ---
SAVE_GCODE_STATE NAME=TOOLCHANGE_STATE

M83
G92 E0

T{next_extruder}

; Purga lógica do slicer
G1 E{flush_length} F300

; Finaliza purga mecânica
CFS_PURGE_FINISH

RESTORE_GCODE_STATE NAME=TOOLCHANGE_STATE MOVE=1
; --- Fim troca filamento ---
