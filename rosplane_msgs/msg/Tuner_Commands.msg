float32 Va_c		# Commanded airspeed (m/s)
float32 h_c		    # Commanded altitude (m)
float32 vh_c        # Commanded vertical velocity (m/s)
float32 chi_c		# Commanded course (rad)
float32 phi_ff		# feed forward command for orbits (rad)
float32[4] aux		# Optional auxiliary commands
bool aux_valid		# Auxiliary commands valid
float32 phi_c       # Commanded roll
float32 theta_c     # Commanded pitch
bool hold_roll      # Set true to tune roll params
bool hold_pitch     # Set true to tune pitch params
bool hold_course    # Set true to tune course params
bool hold_altitude  # Set true to tune altitude params
bool hold_vh        # Set true to tune vertical velocity params