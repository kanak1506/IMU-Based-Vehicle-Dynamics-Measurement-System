# Widget session-state key constants.
# Every module that reads or writes session state imports from here.

# Vehicle configuration widget keys
NAME        = "cfg_name"
MASS        = "cfg_mass"
WHEELBASE   = "cfg_wheelbase"
CG_HEIGHT   = "cfg_cg_height"
CG_FRONT    = "cfg_cg_front"
TRACK_WIDTH = "cfg_track_width"
FRONT_MASS  = "cfg_front_mass"

# Test control widget keys
FILENAME = "cfg_filename"
PORT     = "cfg_port"
RADIUS   = "cfg_radius"

# Internal messaging key (set by callbacks, consumed once by render)
PRESET_MSG = "_preset_msg"

# Logger state key — holds a runner.LogState object
LOGGER_STATE = "_logger_state"

# Pipeline state key — holds a pipeline.PipelineState object
PIPELINE_STATE = "_pipeline_state"

# Maps widget key -> JSON key used when saving/loading presets
# and when passing vehicle params to the analysis pipeline.
VEHICLE_FIELDS: dict[str, str] = {
    NAME:        "name",
    MASS:        "mass_kg",
    WHEELBASE:   "wheelbase_m",
    CG_HEIGHT:   "cg_height_m",
    CG_FRONT:    "cg_to_front_m",
    TRACK_WIDTH: "track_width_m",
    FRONT_MASS:  "front_mass_kg",
}
