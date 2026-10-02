TCHAR * _rs274ngc_errors[] = {
/*   0 */ _T("No error"),
/*   1 */ _T("No error"),
/*   2 */ _T("No error"),
/*   3 */ _T("No error"),
/*   4 */ _T("A file is already open"), // rs274ngc_open
/*   5 */ _T("All axes missing with either g52 or g92"), // enhance_block
/*   6 */ _T("All axes missing with motion code"), // enhance_block
/*   7 */ _T("Arc radius too small to reach end point"), // arc_data_r
/*   8 */ _T("Argument to acos out of range"), // execute_unary
/*   9 */ _T("Argument to asin out of range"), // execute_unary
/*  10 */ _T("Attempt to divide by zero"), // execute_binary1
/*  11 */ _T("Attempt to raise negative to non integer power"), // execute_binary1
/*  12 */ _T("Bad character used"), // read_one_item
/*  13 */ _T("Bad format unsigned integer"), // read_integer_unsigned
/*  14 */ _T("Bad number format"), // read_real_number
/*  15 */ _T("Bug bad g code modal group 0"), // check_g_codes
/*  16 */ _T("Bug code not g0 or g1"), // convert_straight, convert_straight_comp1, convert_straight_comp2
/*  17 */ _T("Bug code not g17 g18 or g19"), // convert_set_plane
/*  18 */ _T("Bug code not g20 or g21"), // convert_length_units
/*  19 */ _T("Bug code not g28 or g30"), // convert_home
/*  20 */ _T("Bug code not g2 or g3"), // arc_data_comp_ijk, arc_data_ijk
/*  21 */ _T("Bug code not g40 g41 or g42"), // convert_cutter_compensation
/*  22 */ _T("Bug code not g43 or g49"), // convert_tool_length_offset
/*  23 */ _T("Bug code not g4 g10 g28 g30 g53 or g92 series"), // convert_modal_0
/*  24 */ _T("Bug code not g61 g61 1 or g64"), // convert_control_mode
/*  25 */ _T("Bug code not g90 or g91"), // convert_distance_mode
/*  26 */ _T("Bug code not g93 or g94 or g95"), // convert_feed_mode
/*  27 */ _T("Bug code not g98 or g99"), // convert_retract_mode
/*  28 */ _T("Bug code not in g92 series"), // convert_axis_offsets
/*  29 */ _T("Bug code not in range g54 to g593"), // convert_coordinate_system
/*  30 */ _T("Bug code not m0 m1 m2 m30 m47 m60"), // convert_stop
/*  31 */ _T("Bug distance mode not g90 or g91"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/*  32 */ _T("Bug function should not have been called"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx, read_a, read_b, read_c, read_comment, read_d, read_f, read_g, read_h, read_i, read_j, read_k, read_l, read_line_number, read_m, read_p, read_parameter, read_parameter_setting, read_q, read_r, read_real_expression, read_s, read_t, read_x, read_y, read_z
/*  33 */ _T("Bug in tool radius comp"), // arc_data_comp_r
/*  34 */ _T("Bug plane not xy yz or xz"), // convert_arc, convert_cycle
/*  35 */ _T("Bug side not right or left"), // convert_straight_comp1, convert_straight_comp2
/*  36 */ _T("Bug unknown motion code"), // convert_motion
/*  37 */ _T("Bug unknown operation"), // execute_binary1, execute_binary2, execute_unary
/*  38 */ _T("Cannot change axis offsets with cutter radius comp"), // convert_axis_offsets
/*  39 */ _T("Cannot change units with cutter radius comp"), // convert_length_units
/*  40 */ _T("Cannot create backup file"), // rs274ngc_save_parameters
/*  41 */ _T("Cannot do g1 with zero feed rate"), // convert_straight
/*  42 */ _T("Cannot do zero repeats of cycle"), // convert_cycle
/*  43 */ _T("Cannot make arc with zero feed rate"), // convert_arc
/*  44 */ _T("Cannot move rotary axes during probing"), // convert_probe
/*  45 */ _T("Cannot open backup file"), // rs274ngc_save_parameters
/*  46 */ _T("Cannot open variable file"), // rs274ngc_save_parameters
/*  47 */ _T("Cannot probe in inverse time feed mode"), // convert_probe
/*  48 */ _T("Cannot probe with cutter radius comp on"), // convert_probe
/*  49 */ _T("Cannot probe with zero feed rate"), // convert_probe
/*  50 */ _T("Cannot put a b in canned cycle"), // check_other_codes
/*  51 */ _T("Cannot put a c in canned cycle"), // check_other_codes
/*  52 */ _T("Cannot put an a in canned cycle"), // check_other_codes
/*  53 */ _T("Cannot turn cutter radius comp on out of xy plane"), // convert_cutter_compensation_on
/*  54 */ _T("Cannot turn cutter radius comp on when on"), // convert_cutter_compensation_on
/*  55 */ _T("Cannot use a word"), // read_a
/*  56 */ _T("Cannot use axis values with g80"), // enhance_block
/*  57 */ _T("Cannot use axis values without a g code that uses them"), // enhance_block
/*  58 */ _T("Cannot use b word"), // read_b
/*  59 */ _T("Cannot use c word"), // read_c
/*  60 */ _T("Cannot use g28 or g30 with cutter radius comp"), // convert_home
/*  61 */ _T("Cannot use g53 incremental"), // check_g_codes
/*  62 */ _T("Cannot use g53 with cutter radius comp"), // convert_straight
/*  63 */ _T("Cannot use two g codes that both use axis values"), // enhance_block
/*  64 */ _T("Cannot use xz plane with cutter radius comp"), // convert_set_plane
/*  65 */ _T("Cannot use yz plane with cutter radius comp"), // convert_set_plane
/*  66 */ _T("Command too long"), // read_text, rs274ngc_open
/*  67 */ _T("Concave corner with cutter radius comp\r\rTo allow Concave Corners enable:\rTool Setup | Trajectory Planner | Allow Concave Corners"), // convert_straight_comp2
/*  68 */ _T("Coordinate system index parameter 5220 out of range"), // rs274ngc_init
/*  69 */ _T("Current point same as end point of arc"), // arc_data_r
/*  70 */ _T("Cutter gouging with cutter radius comp"), // convert_arc_comp1, convert_straight_comp1
/*  71 */ _T("D word with no g41, g42, or g96"), // check_other_codes
/*  72 */ _T("Dwell time missing with g4"), // check_g_codes
/*  73 */ _T("Dwell time p word missing with g82"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/*  74 */ _T("Dwell time p word missing with g86"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/*  75 */ _T("Dwell time p word missing with g88"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/*  76 */ _T("Dwell time p word missing with g89"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/*  77 */ _T("Equal sign missing in parameter setting"), // read_parameter_setting
/*  78 */ _T("F word missing with inverse time arc move"), // convert_arc
/*  79 */ _T("F word missing with inverse time g1 move"), // convert_straight
/*  80 */ _T("File ended with no percent sign"), // read_text, rs274ngc_open
/*  81 */ _T("File ended with no percent sign or program end"), // read_text
/*  82 */ _T("File name too long"), // rs274ngc_open
/*  83 */ _T("File not open"), // rs274ngc_read
/*  84 */ _T("G code out of range"), // read_g
/*  85 */ _T("H word with no g43 or g43.4"), // check_other_codes
/*  86 */ _T("I word given for arc in yz plane"), // convert_arc
/*  87 */ _T("I word missing with g87"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/*  88 */ _T("I word with no g2 or g3 or g87 to use it"), // check_other_codes
/*  89 */ _T("J word given for arc in xz plane"), // convert_arc
/*  90 */ _T("J word missing with g87"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/*  91 */ _T("J word with no g2 or g3 or g87 to use it"), // check_other_codes
/*  92 */ _T("K word given for arc in xy plane"), // convert_arc
/*  93 */ _T("K word missing with g87"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/*  94 */ _T("K word with no g2 or g3 or g83 or g87 to use it"), // check_other_codes
/*  95 */ _T("L word with no canned cycle or g10 or M98"), // check_other_codes
/*  96 */ _T("Left bracket missing after slash with atan"), // read_atan
/*  97 */ _T("Left bracket missing after unary operation name"), // read_unary
/*  98 */ _T("Line number greater than 99999"), // read_line_number
/*  99 */ _T("Line with g10 does not have l2"), // check_g_codes
/* 100 */ _T("M code greater than 119"), // read_m
/* 101 */ _T("Mixed radius ijk format for arc"), // convert_arc
/* 102 */ _T("Multiple a words on one line"), // read_a
/* 103 */ _T("Multiple b words on one line"), // read_b
/* 104 */ _T("Multiple c words on one line"), // read_c
/* 105 */ _T("Multiple d words on one line"), // read_d
/* 106 */ _T("Multiple f words on one line"), // read_f
/* 107 */ _T("Multiple h words on one line"), // read_h
/* 108 */ _T("Multiple i words on one line"), // read_i
/* 109 */ _T("Multiple j words on one line"), // read_j
/* 110 */ _T("Multiple k words on one line"), // read_k
/* 111 */ _T("Multiple l words on one line"), // read_l
/* 112 */ _T("Multiple p words on one line"), // read_p
/* 113 */ _T("Multiple q words on one line"), // read_q
/* 114 */ _T("Multiple r words on one line"), // read_r
/* 115 */ _T("Multiple s words on one line"), // read_s
/* 116 */ _T("Multiple t words on one line"), // read_t
/* 117 */ _T("Multiple x words on one line"), // read_x
/* 118 */ _T("Multiple y words on one line"), // read_y
/* 119 */ _T("Multiple z words on one line"), // read_z
/* 120 */ _T("Must use g0 or g1 with g53"), // check_g_codes
/* 121 */ _T("Negative argument to sqrt"), // execute_unary
/* 122 */ _T("Negative d word tool radius index used"), // read_d
/* 123 */ _T("Negative f word used"), // read_f
/* 124 */ _T("Negative g code used"), // read_g
/* 125 */ _T("Negative h word tool length offset index used"), // read_h
/* 126 */ _T("Negative l word used"), // read_l
/* 127 */ _T("Negative m code used"), // read_m
/* 128 */ _T("Negative q value used"), // read_q
/* 129 */ _T("Negative p word used"), // read_p
/* 130 */ _T("Negative spindle speed used"), // read_s
/* 131 */ _T("Negative tool id used"), // read_t
/* 132 */ _T("Nested comment found"), // close_and_downcase
/* 133 */ _T("No characters found in reading real value"), // read_real_value
/* 134 */ _T("No digits found where real number should be"), // read_real_number
/* 135 */ _T("Non integer value for integer"), // read_integer_value
/* 136 */ _T("Null missing after newline"), // close_and_downcase
/* 137 */ _T("Offset index missing"), // convert_tool_length_offset
/* 138 */ _T("P value not an integer with g10 l2 M98"), // check_g_codes
/* 139 */ _T("P value out of range with g10 l2"), // check_g_codes
/* 140 */ _T("P word with no g4 g10 g49 g82 g83 g86 g88 g89 M98 M100-119"), // check_other_codes
/* 141 */ _T("Parameter file out of order"), // rs274ngc_restore_parameters, rs274ngc_save_parameters
/* 142 */ _T("Parameter number out of range"), // read_parameter, read_parameter_setting, rs274ngc_restore_parameters, rs274ngc_save_parameters
/* 143 */ _T("Q word missing with g83"), // convert_cycle_xy, convert_cycle_yz, convert_cycle_zx
/* 144 */ _T("Q word with no g83, g94, M98, or M100-119"), // check_other_codes
/* 145 */ _T("Queue is not empty after probing"), // rs274ngc_read
/* 146 */ _T("R clearance plane unspecified in cycle"), // convert_cycle
/* 147 */ _T("R i j k words all missing for arc"), // convert_arc
/* 148 */ _T("R less than x in cycle in yz plane"), // convert_cycle_yz
/* 149 */ _T("R less than y in cycle in xz plane"), // convert_cycle_zx
/* 150 */ _T("R less than z in cycle in xy plane"), // convert_cycle_xy
/* 151 */ _T("R word with no g code or m code that uses it"), // check_other_codes
/* 152 */ _T("Radius to end of arc differs from radius to start"), // arc_data_comp_ijk, arc_data_ijk
/* 153 */ _T("Radius too small to reach end point"), // arc_data_comp_r
/* 154 */ _T("Required parameter missing"), // rs274ngc_restore_parameters
/* 155 */ _T("Selected tool slot number too large"), // convert_tool_select
/* 156 */ _T("Slash missing after first atan argument"), // read_atan
/* 157 */ _T("Spindle not turning clockwise in g84"), // convert_cycle_g84
/* 158 */ _T("Spindle not turning in g86"), // convert_cycle_g86
/* 159 */ _T("Spindle not turning in g87"), // convert_cycle_g87
/* 160 */ _T("Spindle not turning in g88"), // convert_cycle_g88
/* 161 */ _T("Sscanf failed"), // read_integer_unsigned, read_real_number
/* 162 */ _T("Start point too close to probe point"), // convert_probe
/* 163 */ _T("Too many m codes on line"), // check_m_codes
/* 164 */ _T("Tool length offset index too big"), // read_h
/* 165 */ _T("Tool max too large"), // rs274ngc_load_tool_table
/* 166 */ _T("Tool radius index too big"), // read_d
/* 167 */ _T("Tool radius not less than arc radius with comp"), // arc_data_comp_r, convert_arc_comp2
/* 168 */ _T("Two g codes used from same modal group"), // read_g
/* 169 */ _T("Two m codes used from same modal group"), // read_m
/* 170 */ _T("Unable to open file"), // convert_stop, rs274ngc_open, rs274ngc_restore_parameters
/* 171 */ _T("Unclosed comment found"), // close_and_downcase
/* 172 */ _T("Unclosed expression"), // read_operation
/* 173 */ _T("Unknown g code used"), // read_g
/* 174 */ _T("Unknown m code used"), // read_m
/* 175 */ _T("Unknown operation"), // read_operation
/* 176 */ _T("Unknown operation name starting with a"), // read_operation
/* 177 */ _T("Unknown operation name starting with m"), // read_operation
/* 178 */ _T("Unknown operation name starting with o"), // read_operation
/* 179 */ _T("Unknown operation name starting with x"), // read_operation
/* 180 */ _T("Unknown word starting with a"), // read_operation_unary
/* 181 */ _T("Unknown word starting with c"), // read_operation_unary
/* 182 */ _T("Unknown word starting with e"), // read_operation_unary
/* 183 */ _T("Unknown word starting with f"), // read_operation_unary
/* 184 */ _T("Unknown word starting with l"), // read_operation_unary
/* 185 */ _T("Unknown word starting with r"), // read_operation_unary
/* 186 */ _T("Unknown word starting with s"), // read_operation_unary
/* 187 */ _T("Unknown word starting with t"), // read_operation_unary
/* 188 */ _T("Unknown word where unary operation could be"), // read_operation_unary
/* 189 */ _T("X and y words missing for arc in xy plane"), // convert_arc
/* 190 */ _T("X and z words missing for arc in xz plane"), // convert_arc
/* 191 */ _T("X value unspecified in yz plane canned cycle"), // convert_cycle_yz
/* 192 */ _T("X y and z words all missing with g38 2"), // convert_probe
/* 193 */ _T("Y and z words missing for arc in yz plane"), // convert_arc
/* 194 */ _T("Y value unspecified in xz plane canned cycle"), // convert_cycle_zx
/* 195 */ _T("Z value unspecified in xy plane canned cycle"), // convert_cycle_xy
/* 196 */ _T("Zero or negative argument to ln"), // execute_unary
/* 197 */ _T("Zero radius arc"), // arc_data_ijk
/* 198 */ _T("Stack Overflow"), // Subroutine Stack
/* 199 */ _T("Subroutine not found"), // Subroutine Lable search
/* 200 */ _T("Stack Underflow - Sub return before call"), // Stack underflow
/* 201 */ _T("Invalid Subroutine Label - only Oxxx and simple comment allowed"), 
/* 202 */ _T("Q value not an integer with M98"),
/* 203 */ _T("L value not an integer with M98"),
/* 204 */ _T("Bug code not g96 or g97"),
/* 205 */ _T("Cannot do g32 with zero feed rate"), // convert_thread
/* 206 */ _T("Cannot use g32 with cutter radius comp"), // convert_thread
/* 207 */ _T("Tool ID not found in Tool Table"), // convert_tool_select
/* 208 */ _T("Tool Slot not found in Tool Table"), // convert_tool_select
/* 209 */ _T("Cannot put a u in canned cycle"), // check_other_codes
/* 210 */ _T("Cannot put a v in canned cycle"), // check_other_codes
/* 211 */ _T("Multiple u words on one line"), // read_u
/* 212 */ _T("Multiple v words on one line"), // read_v
/* 213 */ _T("Invalid Q Value in G83 Canned Cycle"), // convert_cycle_g83
/* 214 */ _T("P word not 1 or 2 with M49"), // convert_m
/* 215 */ _T("g41/g42 Concave Corner : Segment 1 too short (P1 ~= P2)"),// 1 = Segment 1 too short (P1 ~= P2)
/* 216 */ _T("g41/g42 Concave Corner : Segment 2 too short (P2 ~= P3)"),// 2 = Segment 2 too short (P2 ~= P3)
/* 217 */ _T("g41/g42 Concave Corner : Segments nearly parallel, no fillet"),// 3 = Segments nearly parallel, no fillet
/* 218 */ _T("g41/g42 Concave Corner : Segment 2 too short to define tool stop (tangent outside)"),// 4 = Segment 2 too short to define tool stop (tangent outside)
/* 219 */ _T("g41/g42 Concave Corner : Segment 1 too short to define tool start (tangent outside)"),// 5 = Segment 1 too short to define tool start (tangent outside)
/* 220 */ _T("Concave corner caused by Arc with cutter radius comp"), // convert_arc_comp2

_T("The End")};


