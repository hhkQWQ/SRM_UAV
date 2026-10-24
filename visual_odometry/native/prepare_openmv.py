"""Connect rm_vo to the OpenMV v4.5.9 STM32 build, inside this project only.

Usage: python -B native/prepare_openmv.py native/_build/openmv
This prepares sources; it neither downloads, builds nor flashes firmware.
"""
import argparse
from pathlib import Path
import shutil

NATIVE = Path(__file__).resolve().parent
PROJECT = NATIVE.parent.resolve()
MARKER = '# RM_VO single-core native module'


def prepare(root):
    root = Path(root).resolve()
    if root == PROJECT or not root.is_relative_to(PROJECT):
        raise ValueError('OpenMV checkout must be inside the visual odometry folder')
    makefile = root/'src'/'Makefile'
    portconfig = root/'src'/'omv'/'ports'/'stm32'/'omv_portconfig.mk'
    make = makefile.read_text(encoding='utf-8')
    port = portconfig.read_text(encoding='utf-8')
    if 'USER_C_MODULES=$(TOP_DIR)/$(OMV_DIR)' not in make:
        raise ValueError('Unsupported OpenMV layout: expected v4.5.9 USER_C_MODULES')
    anchor = 'FIRM_OBJ += $(wildcard $(BUILD)/$(MICROPY_DIR)/modules/*.o)'
    if anchor not in port:
        raise ValueError('Unsupported OpenMV STM32 link layout')
    destination = root/'src'/'omv'/'rm_vo'
    # Resolve every write target before copying, including existing symlinks.
    targets = [destination/name for name in ('modvo.c','vo_core.c','vo_core.h','vo_fast.c','vo_fast.h','micropython.mk')]
    for target in targets+[portconfig]:
        if not target.resolve().is_relative_to(PROJECT):
            raise ValueError('Write target escapes project: '+str(target))
    # Replace our own block on upgrades as well as first install.
    if MARKER in port:
        begin=port.index(MARKER)
        end=port.index('\n\n',begin)+2
        port=port[:begin]+port[end:]
    snippet=(MARKER+'\n'+''.join('FIRM_OBJ += $(BUILD)/$(MICROPY_DIR)/rm_vo/'+name+'.o\n'
                                 for name in ('modvo','vo_core','vo_fast'))+'\n')
    port=port.replace(anchor,snippet+anchor,1)
    destination.mkdir(parents=True,exist_ok=True)
    for target in targets:
        shutil.copyfile(NATIVE/target.name,target)
    portconfig.write_text(port,encoding='utf-8')
    print('Prepared rm_vo for OpenMV v4.5.9:',root)
    print('Build from src using the actual board TARGET (H7 Plus: OPENMV4P).')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('openmv',help='OpenMV v4.5.9 checkout inside this project')
    prepare(parser.parse_args().openmv)
