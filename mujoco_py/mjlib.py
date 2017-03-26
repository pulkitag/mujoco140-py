from ctypes import *
import os
from .util import *
from .mjtypes import *

from mujoco_py import config

path_prefix = config.mjpro_path
if sys.platform.startswith("darwin"):
    libfile = os.path.join(path_prefix, "bin/libmujoco131.dylib")
elif sys.platform.startswith("linux"):
    #libfile = os.path.join(path_prefix, "bin/libmujoco131.so")
    libfile = os.path.join(path_prefix, "bin/libmujoco140nogl.so")
elif sys.platform.startswith("win"):
    libfile = os.path.join(path_prefix, "bin/mujoco131.lib")
else:
    raise RuntimeError("Unrecognized platform %s" % sys.platform)

if not os.path.exists(libfile):
    raise RuntimeError("Missing path: %s. (HINT: you should have unzipped the mjpro131.zip bundle without modification.)" % libfile)

mjlib = cdll.LoadLibrary(os.path.abspath(libfile))

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 87
mjlib.mj_activate.argtypes = [String]
mjlib.mj_activate.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 90
mjlib.mj_deactivate.argtypes = []
mjlib.mj_deactivate.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 93
mjlib.mj_certQuestion.argtypes = [c_double * 16]
mjlib.mj_certQuestion.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 96
mjlib.mj_certAnswer.argtypes = [c_double * 16, c_double * 16]
mjlib.mj_certAnswer.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 99
mjlib.mj_certCheck.argtypes = [c_double * 16, c_double * 16]
mjlib.mj_certCheck.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 107
mjlib.mj_loadXML.argtypes = [String, String, String, c_int]
mjlib.mj_loadXML.restype = POINTER(mjModel)

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 112
mjlib.mj_saveXML.argtypes = [String, POINTER(MJMODEL), String, c_int]
mjlib.mj_saveXML.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 115
mjlib.mj_printSchema.argtypes = [String, String, c_int, c_int, c_int]
mjlib.mj_printSchema.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 122
mjlib.mj_step.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_step.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 125
mjlib.mj_step1.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_step1.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 128
mjlib.mj_step2.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_step2.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 131
mjlib.mj_forward.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_forward.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 134
mjlib.mj_inverse.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_inverse.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 137
mjlib.mj_forwardSkip.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int, c_int]
mjlib.mj_forwardSkip.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 141
mjlib.mj_inverseSkip.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int, c_int]
mjlib.mj_inverseSkip.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 148
mjlib.mj_defaultSolRefImp.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mj_defaultSolRefImp.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 151
mjlib.mj_defaultOption.argtypes = [POINTER(MJOPTION)]
mjlib.mj_defaultOption.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 154
mjlib.mj_defaultVisual.argtypes = [POINTER(MJVISUAL)]
mjlib.mj_defaultVisual.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 157
mjlib.mj_copyModel.argtypes = [POINTER(MJMODEL), POINTER(MJMODEL)]
mjlib.mj_copyModel.restype = POINTER(mjModel)

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 160
mjlib.mj_saveModel.argtypes = [POINTER(MJMODEL), String, POINTER(None), c_int]
mjlib.mj_saveModel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 163
mjlib.mj_loadModel.argtypes = [String, POINTER(None), c_int]
mjlib.mj_loadModel.restype = POINTER(mjModel)

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 166
mjlib.mj_deleteModel.argtypes = [POINTER(MJMODEL)]
mjlib.mj_deleteModel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 169
mjlib.mj_sizeModel.argtypes = [POINTER(MJMODEL)]
mjlib.mj_sizeModel.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 172
mjlib.mj_makeData.argtypes = [POINTER(MJMODEL)]
mjlib.mj_makeData.restype = POINTER(mjData)

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 175
mjlib.mj_copyData.argtypes = [POINTER(MJDATA), POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_copyData.restype = POINTER(mjData)

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 178
mjlib.mj_resetData.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_resetData.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 181
mjlib.mj_resetDataDebug.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_ubyte]
mjlib.mj_resetDataDebug.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 184
mjlib.mj_resetDataKeyframe.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int]
mjlib.mj_resetDataKeyframe.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 187
mjlib.mj_stackAlloc.argtypes = [POINTER(MJDATA), c_int]
mjlib.mj_stackAlloc.restype = POINTER(mjtNum)

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 190
mjlib.mj_deleteData.argtypes = [POINTER(MJDATA)]
mjlib.mj_deleteData.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 193
mjlib.mj_resetCallbacks.argtypes = []
mjlib.mj_resetCallbacks.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 196
mjlib.mj_setConst.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int]
mjlib.mj_setConst.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 202
mjlib.mj_printModel.argtypes = [POINTER(MJMODEL), String]
mjlib.mj_printModel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 205
mjlib.mj_printData.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), String]
mjlib.mj_printData.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 208
mjlib.mju_printMat.argtypes = [POINTER(c_double), c_int, c_int]
mjlib.mju_printMat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 214
mjlib.mj_fwdPosition.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_fwdPosition.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 217
mjlib.mj_fwdVelocity.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_fwdVelocity.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 220
mjlib.mj_fwdActuation.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_fwdActuation.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 223
mjlib.mj_fwdAcceleration.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_fwdAcceleration.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 226
mjlib.mj_fwdConstraint.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_fwdConstraint.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 229
mjlib.mj_Euler.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_Euler.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 232
mjlib.mj_RungeKutta.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int]
mjlib.mj_RungeKutta.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 238
mjlib.mj_invPosition.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_invPosition.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 241
mjlib.mj_invVelocity.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_invVelocity.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 244
mjlib.mj_invConstraint.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_invConstraint.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 247
mjlib.mj_compareFwdInv.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_compareFwdInv.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 253
mjlib.mj_sensorPos.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_sensorPos.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 256
mjlib.mj_sensorVel.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_sensorVel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 259
mjlib.mj_sensorAcc.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_sensorAcc.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 262
mjlib.mj_energyPos.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_energyPos.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 265
mjlib.mj_energyVel.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_energyVel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 271
mjlib.mj_checkPos.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_checkPos.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 274
mjlib.mj_checkVel.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_checkVel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 277
mjlib.mj_checkAcc.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_checkAcc.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 280
mjlib.mj_kinematics.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_kinematics.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 283
mjlib.mj_comPos.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_comPos.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 286
mjlib.mj_camlight.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_camlight.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 289
mjlib.mj_tendon.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_tendon.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 292
mjlib.mj_transmission.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_transmission.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 295
mjlib.mj_crb.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_crb.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 298
mjlib.mj_factorM.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_factorM.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 301
mjlib.mj_backsubM.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_backsubM.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 304
mjlib.mj_backsubM2.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_backsubM2.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 307
mjlib.mj_comVel.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_comVel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 310
mjlib.mj_passive.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_passive.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 313
mjlib.mj_rne.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int, POINTER(c_double)]
mjlib.mj_rne.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 316
mjlib.mj_rnePostConstraint.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_rnePostConstraint.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 319
mjlib.mj_collision.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_collision.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 322
mjlib.mj_makeConstraint.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_makeConstraint.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 325
mjlib.mj_projectConstraint.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_projectConstraint.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 328
mjlib.mj_referenceConstraint.argtypes = [POINTER(MJMODEL), POINTER(MJDATA)]
mjlib.mj_referenceConstraint.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 334
mjlib.mj_addContact.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(MJCONTACT)]
mjlib.mj_addContact.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 337
mjlib.mj_isPyramid.argtypes = [POINTER(MJMODEL)]
mjlib.mj_isPyramid.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 340
mjlib.mj_isSparse.argtypes = [POINTER(MJMODEL)]
mjlib.mj_isSparse.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 343
mjlib.mj_mulJacVec.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double)]
mjlib.mj_mulJacVec.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 347
mjlib.mj_mulJacTVec.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double)]
mjlib.mj_mulJacTVec.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 350
mjlib.mj_jac.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_jac.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 354
mjlib.mj_jacBody.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_jacBody.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 358
mjlib.mj_jacBodyCom.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_jacBodyCom.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 362
mjlib.mj_jacGeom.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_jacGeom.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 366
mjlib.mj_jacSite.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_jacSite.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 370
mjlib.mj_jacPointAxis.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_jacPointAxis.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 375
mjlib.mj_name2id.argtypes = [POINTER(MJMODEL), c_int, String]
mjlib.mj_name2id.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 378
mjlib.mj_id2name.argtypes = [POINTER(MJMODEL), c_int, c_int]
if sizeof(c_int) == sizeof(c_void_p):
    mjlib.mj_id2name.restype = ReturnString
else:
    mjlib.mj_id2name.restype = String
    mjlib.mj_id2name.errcheck = ReturnString

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 381
mjlib.mj_fullM.argtypes = [POINTER(MJMODEL), POINTER(c_double), POINTER(c_double)]
mjlib.mj_fullM.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 384
mjlib.mj_mulM.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double)]
mjlib.mj_mulM.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 387
mjlib.mj_applyFT.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int, POINTER(c_double)]
mjlib.mj_applyFT.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 392
mjlib.mj_objectVelocity.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int, c_int, POINTER(c_double), c_int]
mjlib.mj_objectVelocity.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 396
mjlib.mj_objectAcceleration.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int, c_int, POINTER(c_double), c_int]
mjlib.mj_objectAcceleration.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 400
mjlib.mj_differentiatePos.argtypes = [POINTER(MJMODEL), POINTER(c_double), c_double, POINTER(c_double), POINTER(c_double)]
mjlib.mj_differentiatePos.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 404
mjlib.mj_contactForce.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int, POINTER(c_double)]
mjlib.mj_contactForce.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 407
mjlib.mj_integratePos.argtypes = [POINTER(MJMODEL), POINTER(c_double), POINTER(c_double), c_double]
mjlib.mj_integratePos.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 410
mjlib.mj_normalizeQuat.argtypes = [POINTER(MJMODEL), POINTER(c_double)]
mjlib.mj_normalizeQuat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 413
mjlib.mj_local2Global.argtypes = [POINTER(MJDATA), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mj_local2Global.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 417
mjlib.mj_getTotalmass.argtypes = [POINTER(MJMODEL)]
mjlib.mj_getTotalmass.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 420
mjlib.mj_setTotalmass.argtypes = [POINTER(MJMODEL), c_double]
mjlib.mj_setTotalmass.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 423
mjlib.mj_version.argtypes = []
mjlib.mj_version.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 429
mjlib.mjv_defaultCamera.argtypes = [POINTER(MJVCAMERA)]
mjlib.mjv_defaultCamera.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 432
mjlib.mjv_defaultPerturb.argtypes = [POINTER(MJVPERTURB)]
mjlib.mjv_defaultPerturb.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 435
mjlib.mjv_room2model.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(MJVSCENE)]
mjlib.mjv_room2model.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 439
mjlib.mjv_model2room.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(MJVSCENE)]
mjlib.mjv_model2room.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 443
mjlib.mjv_cameraInModel.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(MJVSCENE)]
mjlib.mjv_cameraInModel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 446
mjlib.mjv_cameraInRoom.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(MJVSCENE)]
mjlib.mjv_cameraInRoom.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 449
mjlib.mjv_frustumHeight.argtypes = [POINTER(MJVSCENE)]
mjlib.mjv_frustumHeight.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 452
mjlib.mjv_alignToCamera.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mjv_alignToCamera.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 455
mjlib.mjv_moveCamera.argtypes = [POINTER(MJMODEL), c_int, c_double, c_double, POINTER(MJVSCENE), POINTER(MJVCAMERA)]
mjlib.mjv_moveCamera.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 459
mjlib.mjv_movePerturb.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), c_int, c_double, c_double, POINTER(MJVSCENE), POINTER(MJVPERTURB)]
mjlib.mjv_movePerturb.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 463
mjlib.mjv_moveModel.argtypes = [POINTER(MJMODEL), c_int, c_double, c_double, POINTER(c_double), POINTER(MJVSCENE)]
mjlib.mjv_moveModel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 467
mjlib.mjv_initPerturb.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(MJVSCENE), POINTER(MJVPERTURB)]
mjlib.mjv_initPerturb.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 472
mjlib.mjv_applyPerturbPose.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(MJVPERTURB), c_int]
mjlib.mjv_applyPerturbPose.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 476
mjlib.mjv_applyPerturbForce.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(MJVPERTURB)]
mjlib.mjv_applyPerturbForce.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 482
mjlib.mjv_defaultOption.argtypes = [POINTER(MJVOPTION)]
mjlib.mjv_defaultOption.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 485
mjlib.mjv_makeScene.argtypes = [POINTER(MJVSCENE), c_int]
mjlib.mjv_makeScene.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 488
mjlib.mjv_freeScene.argtypes = [POINTER(MJVSCENE)]
mjlib.mjv_freeScene.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 491
mjlib.mjv_updateScene.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(MJVOPTION), POINTER(MJVPERTURB), POINTER(MJVCAMERA), c_int, POINTER(MJVSCENE)]
mjlib.mjv_updateScene.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 495
mjlib.mjv_addGeoms.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(MJVOPTION), POINTER(MJVPERTURB), c_int, POINTER(MJVSCENE)]
mjlib.mjv_addGeoms.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 499
mjlib.mjv_updateCamera.argtypes = [POINTER(MJMODEL), POINTER(MJDATA), POINTER(MJVCAMERA), POINTER(MJVSCENE)]
mjlib.mjv_updateCamera.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 505
mjlib.mjr_defaultContext.argtypes = [POINTER(MJRCONTEXT)]
mjlib.mjr_defaultContext.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 508
mjlib.mjr_makeContext.argtypes = [POINTER(MJMODEL), POINTER(MJRCONTEXT), c_int]
mjlib.mjr_makeContext.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 511
mjlib.mjr_freeContext.argtypes = [POINTER(MJRCONTEXT)]
mjlib.mjr_freeContext.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 514
mjlib.mjr_uploadTexture.argtypes = [POINTER(MJMODEL), POINTER(MJRCONTEXT), c_int]
mjlib.mjr_uploadTexture.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 517
mjlib.mjr_uploadMesh.argtypes = [POINTER(MJMODEL), POINTER(MJRCONTEXT), c_int]
mjlib.mjr_uploadMesh.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 520
mjlib.mjr_uploadHField.argtypes = [POINTER(MJMODEL), POINTER(MJRCONTEXT), c_int]
mjlib.mjr_uploadHField.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 524
mjlib.mjr_setBuffer.argtypes = [c_int, POINTER(MJRCONTEXT)]
mjlib.mjr_setBuffer.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 528
mjlib.mjr_readPixels.argtypes = [POINTER(c_ubyte), POINTER(c_float), MJRRECT, POINTER(MJRCONTEXT)]
mjlib.mjr_readPixels.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 533
mjlib.mjr_drawPixels.argtypes = [POINTER(c_ubyte), POINTER(c_float), MJRRECT, POINTER(MJRCONTEXT)]
mjlib.mjr_drawPixels.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 538
mjlib.mjr_blitBuffer.argtypes = [MJRRECT, MJRRECT, c_int, c_int, POINTER(MJRCONTEXT)]
mjlib.mjr_blitBuffer.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 542
mjlib.mjr_text.argtypes = [c_int, String, POINTER(MJRCONTEXT), c_float, c_float, c_float, c_float, c_float]
mjlib.mjr_text.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 546
mjlib.mjr_overlay.argtypes = [c_int, c_int, MJRRECT, String, String, POINTER(MJRCONTEXT)]
mjlib.mjr_overlay.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 550
mjlib.mjr_maxViewport.argtypes = [POINTER(MJRCONTEXT)]
mjlib.mjr_maxViewport.restype = mjrRect

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 553
mjlib.mjr_rectangle.argtypes = [MJRRECT, c_float, c_float, c_float, c_float]
mjlib.mjr_rectangle.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 556
mjlib.mjr_lines.argtypes = [MJRRECT, c_int, POINTER(c_float), POINTER(c_int), POINTER(c_double)]
mjlib.mjr_lines.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 560
mjlib.mjr_render.argtypes = [MJRRECT, POINTER(MJVSCENE), POINTER(MJRCONTEXT)]
mjlib.mjr_render.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 563
mjlib.mjr_select.argtypes = [MJRRECT, POINTER(MJVSCENE), POINTER(MJRCONTEXT), c_int, c_int, POINTER(c_double), POINTER(c_double)]
mjlib.mjr_select.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 567
mjlib.mjr_finish.argtypes = []
mjlib.mjr_finish.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 570
mjlib.mjr_getError.argtypes = []
mjlib.mjr_getError.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 576
mjlib.mju_error.argtypes = [String]
mjlib.mju_error.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 579
mjlib.mju_error_i.argtypes = [String, c_int]
mjlib.mju_error_i.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 582
mjlib.mju_error_s.argtypes = [String, String]
mjlib.mju_error_s.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 585
mjlib.mju_warning.argtypes = [String]
mjlib.mju_warning.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 588
mjlib.mju_warning_i.argtypes = [String, c_int]
mjlib.mju_warning_i.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 591
mjlib.mju_warning_s.argtypes = [String, String]
mjlib.mju_warning_s.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 594
mjlib.mju_clearHandlers.argtypes = []
mjlib.mju_clearHandlers.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 597
mjlib.mju_malloc.argtypes = [c_size_t]
mjlib.mju_malloc.restype = POINTER(None)

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 600
mjlib.mju_free.argtypes = [POINTER(None)]
mjlib.mju_free.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 603
mjlib.mj_warning.argtypes = [POINTER(MJDATA), c_int, c_int]
mjlib.mj_warning.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 647
mjlib.mju_zero3.argtypes = [POINTER(c_double)]
mjlib.mju_zero3.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 650
mjlib.mju_copy3.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mju_copy3.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 653
mjlib.mju_scl3.argtypes = [POINTER(c_double), POINTER(c_double), c_double]
mjlib.mju_scl3.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 656
mjlib.mju_add3.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_add3.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 659
mjlib.mju_sub3.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_sub3.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 662
mjlib.mju_addTo3.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mju_addTo3.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 665
mjlib.mju_addToScl3.argtypes = [POINTER(c_double), POINTER(c_double), c_double]
mjlib.mju_addToScl3.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 668
mjlib.mju_addScl3.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_double]
mjlib.mju_addScl3.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 671
mjlib.mju_normalize3.argtypes = [POINTER(c_double)]
mjlib.mju_normalize3.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 674
mjlib.mju_norm3.argtypes = [POINTER(c_double)]
mjlib.mju_norm3.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 677
mjlib.mju_dot3.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mju_dot3.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 680
mjlib.mju_dist3.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mju_dist3.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 683
mjlib.mju_rotVecMat.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_rotVecMat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 686
mjlib.mju_rotVecMatT.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_rotVecMatT.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 689
mjlib.mju_cross.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_cross.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 692
mjlib.mju_zero.argtypes = [POINTER(c_double), c_int]
mjlib.mju_zero.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 695
mjlib.mju_copy.argtypes = [POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_copy.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 698
mjlib.mju_scl.argtypes = [POINTER(c_double), POINTER(c_double), c_double, c_int]
mjlib.mju_scl.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 701
mjlib.mju_add.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_add.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 704
mjlib.mju_sub.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_sub.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 707
mjlib.mju_addTo.argtypes = [POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_addTo.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 710
mjlib.mju_addToScl.argtypes = [POINTER(c_double), POINTER(c_double), c_double, c_int]
mjlib.mju_addToScl.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 713
mjlib.mju_addScl.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_double, c_int]
mjlib.mju_addScl.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 717
mjlib.mju_normalize.argtypes = [POINTER(c_double), c_int]
mjlib.mju_normalize.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 720
mjlib.mju_norm.argtypes = [POINTER(c_double), c_int]
mjlib.mju_norm.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 723
mjlib.mju_dot.argtypes = [POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_dot.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 726
mjlib.mju_mulMatVec.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int, c_int]
mjlib.mju_mulMatVec.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 730
mjlib.mju_mulMatTVec.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int, c_int]
mjlib.mju_mulMatTVec.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 734
mjlib.mju_transpose.argtypes = [POINTER(c_double), POINTER(c_double), c_int, c_int]
mjlib.mju_transpose.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 737
mjlib.mju_mulMatMat.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int, c_int, c_int]
mjlib.mju_mulMatMat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 741
mjlib.mju_mulMatMatT.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int, c_int, c_int]
mjlib.mju_mulMatMatT.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 745
mjlib.mju_mulMatTMat.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int, c_int, c_int]
mjlib.mju_mulMatTMat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 749
mjlib.mju_sqrMat.argtypes = [POINTER(c_double), POINTER(c_double), c_int, c_int, POINTER(c_double), c_int]
mjlib.mju_sqrMat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 753
mjlib.mju_sqrMatTD.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int, c_int]
mjlib.mju_sqrMatTD.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 757
mjlib.mju_transformSpatial.argtypes = [POINTER(c_double), POINTER(c_double), c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_transformSpatial.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 765
mjlib.mju_rotVecQuat.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_rotVecQuat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 768
mjlib.mju_negQuat.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mju_negQuat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 771
mjlib.mju_mulQuat.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_mulQuat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 774
mjlib.mju_mulQuatAxis.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_mulQuatAxis.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 777
mjlib.mju_axisAngle2Quat.argtypes = [POINTER(c_double), POINTER(c_double), c_double]
mjlib.mju_axisAngle2Quat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 780
mjlib.mju_quat2Vel.argtypes = [POINTER(c_double), POINTER(c_double), c_double]
mjlib.mju_quat2Vel.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 783
mjlib.mju_quat2Mat.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mju_quat2Mat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 786
mjlib.mju_mat2Quat.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mju_mat2Quat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 789
mjlib.mju_derivQuat.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_derivQuat.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 792
mjlib.mju_quatIntegrate.argtypes = [POINTER(c_double), POINTER(c_double), c_double]
mjlib.mju_quatIntegrate.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 795
mjlib.mju_quatZ2Vec.argtypes = [POINTER(c_double), POINTER(c_double)]
mjlib.mju_quatZ2Vec.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 801
mjlib.mju_mulPose.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_mulPose.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 805
mjlib.mju_negPose.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_negPose.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 809
mjlib.mju_trnVecPose.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_trnVecPose.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 816
mjlib.mju_cholFactor.argtypes = [POINTER(c_double), POINTER(c_double), c_int, c_double, c_double, POINTER(c_double)]
mjlib.mju_cholFactor.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 820
mjlib.mju_cholBacksub.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int, c_int, c_int]
mjlib.mju_cholBacksub.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 824
mjlib.mju_eig3.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double)]
mjlib.mju_eig3.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 830
mjlib.mju_muscleFVL.argtypes = [c_double, c_double, c_double, c_double, POINTER(c_double)]
mjlib.mju_muscleFVL.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 833
mjlib.mju_musclePassive.argtypes = [c_double, c_double, c_double, POINTER(c_double)]
mjlib.mju_musclePassive.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 836
mjlib.mju_pneumatic.argtypes = [c_double, c_double, c_double, POINTER(c_double), c_double, c_double, c_double, POINTER(c_double)]
mjlib.mju_pneumatic.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 840
mjlib.mju_encodePyramid.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_encodePyramid.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 844
mjlib.mju_decodePyramid.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_decodePyramid.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 848
mjlib.mju_springDamper.argtypes = [c_double, c_double, c_double, c_double, c_double]
mjlib.mju_springDamper.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 851
mjlib.mju_min.argtypes = [c_double, c_double]
mjlib.mju_min.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 854
mjlib.mju_max.argtypes = [c_double, c_double]
mjlib.mju_max.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 857
mjlib.mju_sign.argtypes = [c_double]
mjlib.mju_sign.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 860
mjlib.mju_round.argtypes = [c_double]
mjlib.mju_round.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 863
mjlib.mju_type2Str.argtypes = [c_int]
if sizeof(c_int) == sizeof(c_void_p):
    mjlib.mju_type2Str.restype = ReturnString
else:
    mjlib.mju_type2Str.restype = String
    mjlib.mju_type2Str.errcheck = ReturnString

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 866
mjlib.mju_str2Type.argtypes = [String]
mjlib.mju_str2Type.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 869
mjlib.mju_warningText.argtypes = [c_int, c_int]
if sizeof(c_int) == sizeof(c_void_p):
    mjlib.mju_warningText.restype = ReturnString
else:
    mjlib.mju_warningText.restype = String
    mjlib.mju_warningText.errcheck = ReturnString

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 872
mjlib.mju_isBad.argtypes = [c_double]
mjlib.mju_isBad.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 875
mjlib.mju_isZero.argtypes = [POINTER(c_double), c_int]
mjlib.mju_isZero.restype = c_int

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 878
mjlib.mju_standardNormal.argtypes = [POINTER(c_double)]
mjlib.mju_standardNormal.restype = mjtNum

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 881
mjlib.mju_f2n.argtypes = [POINTER(c_double), POINTER(c_float), c_int]
mjlib.mju_f2n.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 884
mjlib.mju_n2f.argtypes = [POINTER(c_float), POINTER(c_double), c_int]
mjlib.mju_n2f.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 887
mjlib.mju_d2n.argtypes = [POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_d2n.restype = None

# /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h: 890
mjlib.mju_n2d.argtypes = [POINTER(c_double), POINTER(c_double), c_int]
mjlib.mju_n2d.restype = None


