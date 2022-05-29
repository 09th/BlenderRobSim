import os, bpy

header_src = '''&ACCESS RVP
&REL 4
&PARAM EDITMASK = *
&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe
&PARAM DISKPATH = KRC:\\R1\\Program\\test\\test_search
DEF tst_main( )
;FOLD INI;%{PE}
  ;FOLD BASISTECH INI
    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
    INTERRUPT ON 3 
    BAS (#INITMOV,0 )
  ;ENDFOLD (BASISTECH INI)
  ;FOLD USER INI
    ;Make your modifications here

  ;ENDFOLD (USER INI)
;ENDFOLD (INI)
'''

header_dat='''&ACCESS RVP
&REL 4
&PARAM EDITMASK = *
&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe
&PARAM DISKPATH = KRC:\\R1\\Program\\test\\test_search
DEFDAT  TST_MAIN PUBLIC
;FOLD EXTERNAL DECLARATIONS;%{PE}%MKUKATPBASIS,%CEXT,%VCOMMON,%P
;FOLD BASISTECH EXT;%{PE}%MKUKATPBASIS,%CEXT,%VEXT,%P
EXT  BAS (BAS_COMMAND  :IN,REAL  :IN )
DECL INT SUCCESS
;ENDFOLD (BASISTECH EXT)
;FOLD USER EXT;%{E}%MKUKATPUSER,%CEXT,%VEXT,%P
;Make your modifications here

;ENDFOLD (USER EXT)
;ENDFOLD (EXTERNAL DECLARATIONS)
'''

footer_src = 'END'
footer_dat = 'ENDDAT'

#['P1', 25.0, 'PDAT1', <tool>, <base>]
ptp_tmpl_src = ''';FOLD PTP {0} Vel={1} % {2} Tool[{3}] Base[{4}];%{{PE}}
;FOLD Parameters ;%{{h}}
;Params IlfProvider=kukaroboter.basistech.inlineforms.movement.old; Kuka.IsGlobalPoint=False; Kuka.PointName={0}; Kuka.BlendingEnabled=False; Kuka.MoveDataPtpName={2}; Kuka.VelocityPtp=100; Kuka.CurrentCDSetIndex=0; Kuka.MovementParameterFieldEnabled=True; IlfCommand=PTP
;ENDFOLD
$BWDSTART = FALSE
PDAT_ACT = P{2}
FDAT_ACT = F{0}
BAS(#PTP_PARAMS, {1})
SET_CD_PARAMS (0)
PTP X{0}
;ENDFOLD
'''

ptp_tmpl_dat = '''DECL E6AXIS X{0}={{A1 {5:.3f},A2 {6:.3f},A3 {7:.3f},A4 {8:.3f},A5 {9:.3f},A6 {10:.3f},E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}}
DECL FDAT F{0}={{TOOL_NO {3},BASE_NO {4},IPO_FRAME #BASE,POINT2[] " "}}
DECL PDAT P{2}={{VEL {1},ACC 100.000,APO_DIST 500.000,APO_MODE #CDIS,GEAR_JERK 100.000,EXAX_IGN 0}}
'''

lin_tmpl_src = ''';FOLD LIN {0} Vel={1} m/s {2} Tool[{3}] Base[{4}] ;%{{PE}}
;FOLD Parameters ;%{{h}}
;Params IlfProvider=kukaroboter.basistech.inlineforms.movement.old; Kuka.IsGlobalPoint=False; Kuka.PointName={0}; Kuka.BlendingEnabled=False; Kuka.MoveDataName={2}; Kuka.VelocityPath={1}; Kuka.CurrentCDSetIndex=0; Kuka.MovementParameterFieldEnabled=True; IlfCommand=LIN
;ENDFOLD
$BWDSTART = FALSE
LDAT_ACT = L{2}
FDAT_ACT = F{0}
BAS(#CP_PARAMS, {1})
SET_CD_PARAMS (0)
LIN X{0}
;ENDFOLD
'''

lin_tmpl_dat = '''DECL E6POS X{0}={{X {5:.3f},Y {6:.3f},Z {7:.3f},A {8:.3f},B {9:.3f},C {10:.3f},S 22,T 50,E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}}
DECL FDAT F{0}={{TOOL_NO {3},BASE_NO {4},IPO_FRAME #BASE,POINT2[] " "}}
DECL LDAT L{2}={{VEL {1},ACC 100.000,APO_DIST 500.000,APO_FAC 50.0000,AXIS_VEL 100.000,AXIS_ACC 100.000,ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0000,GEAR_JERK 100.000,EXAX_IGN 0}}
'''

mod_rp = bpy.data.texts['rob_parameters'].as_module()

def save_KRL(sequence, fname):
    tool = 3
    base = 0
    src = header_src
    dat = header_dat
    p_num = 1
    for waypoint in sequence:
        pn = 'P{0}'.format(p_num)
        pdat = 'PDAT{0}'.format(p_num)
        ldat = 'CPDAT{0}'.format(p_num)
        pname, p, spd, axis, t, stored_mat = waypoint
        if pname.startswith('PTP'):
            pp = [pn, spd, pdat, tool, base]
            ax_val = [axis[an] for an in mod_rp.parameters['axis_names']]
            src += ptp_tmpl_src.format(*pp)
            dat += ptp_tmpl_dat.format(*(pp + ax_val))
        elif pname.startswith('LIN'):
            mat = p.matrix_world
            lp = [pn, spd, ldat, tool, base]
            tr = mat.translation
            eul = mat.to_euler()
            frame = [tr.x*1000, tr.y*1000, tr.z*1000, eul.z, eul.y, eul.x]
            src += lin_tmpl_src.format(*lp)
            dat += lin_tmpl_dat.format(*(lp + frame))
        p_num += 1
    src += footer_src
    dat += footer_dat
    src_fname = '{0}\\{1}.src'.format(os.path.dirname(bpy.data.filepath), fname)
    dat_fname = '{0}\\{1}.dat'.format(os.path.dirname(bpy.data.filepath), fname)
    with open(src_fname, 'w') as f:
        f.write(src)
    with open(dat_fname, 'w') as f:
        f.write(dat)
        

#p = ['P1', 25.0, 'PDAT1', 3, 0]
#print(ptp_tmpl_src.format(*p))
#a = [10,20,30,40,50,60]
#print(ptp_tmpl_dat.format(*(p + a)))

#print(lin_tmpl_src.format(*p))

#print(lin_tmpl_dat.format(*(p + a)))
