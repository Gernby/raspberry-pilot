#!/usr/bin/env python3
# simple boardd wrapper that updates the panda first
import os
import time
from selfdrive.kegman_conf import kegman_conf

from selfdrive.swaglog import cloudlog
from panda import Panda, PandaDFU, BASEDIR
from setproctitle import setproctitle


def update_panda():
  with open(os.path.join(BASEDIR, "VERSION")) as f:
    repo_version = f.read()
  repo_version += "-EON" if os.path.isfile('/EON') else "-DEV"

  panda = None
  panda_dfu = None

  print("Connecting to panda")

  while True:
    # break on normal mode Panda
    panda_list = Panda.list()
    if len(panda_list) > 0:
      print("Panda found, connecting")
      panda = Panda(panda_list[0])
      break

    # flash on DFU mode Panda
    panda_dfu = PandaDFU.list()
    if len(panda_dfu) > 0:
      print("Panda in DFU mode found, flashing recovery")
      panda_dfu = PandaDFU(panda_dfu[0])
      panda_dfu.recover()

    #print("waiting for board..."
    time.sleep(1)

  current_version = "bootstub" if panda.bootstub else str(panda.get_version())
  print("Panda connected, version: %s, expected %s" % (current_version, repo_version))

  if panda.bootstub or not current_version.startswith(repo_version):
    print("Panda firmware out of date, update required")

    signed_fn = os.path.join(BASEDIR, "board", "obj", "panda.bin.signed")
    if os.path.exists(signed_fn):
      print("Flashing signed firmware")
      panda.flash(fn=signed_fn)
    else:
      print("Building and flashing unsigned firmware")
      panda.flash()

    print("Done flashing")

  if panda.bootstub:
    print("Flashed firmware not booting, flashing development bootloader")
    panda.recover()
    print("Done flashing bootloader")

  if panda.bootstub:
    print("Panda still not booting, exiting")
    raise AssertionError

  version = str(panda.get_version())
  if not version.startswith(repo_version):
    print("Version mismatch after flashing, exiting")
    raise AssertionError


def main(gctx=None):
  setproctitle('pandad')
  try:
    kegman = kegman_conf()  #.read_config()
    if bool(kegman.conf['useAutoFlash']): 
      update_panda()
  except:
    pass

  os.chdir("selfdrive/boardd")
  os.execvp("./boardd", ["./boardd"])

if __name__ == "__main__":
  main()
