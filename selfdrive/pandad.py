#!/usr/bin/env python3
# simple boardd wrapper that updates the panda first
import os
import subprocess
import time
from selfdrive.kegman_conf import kegman_conf
from common.params import Params, put_nonblocking
from selfdrive.swaglog import cloudlog
from panda import Panda, PandaDFU, BASEDIR, PandaSerial, PandaWifiStreaming
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
    for i in range(len(panda_list)):
    #if len(panda_list) > 0:
      print("Panda found, connecting")
      panda = Panda(panda_list[i])
      if panda.is_white():
        print("white", panda.health())
      elif panda.is_grey():
        print("gray", panda.health())
      else:
        print("black", panda.health())

    if len(panda_list) > 0: break
    
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

  if panda.is_white():
    print("white", panda.health())
  elif panda.is_grey():
    print("gray", panda.health())
  else:
    print("black", panda.health())


  if panda.bootstub or not current_version.startswith(repo_version):
    print("Panda firmware out of date, update required")

    if panda.is_black() or (panda.health()['current'] > 2000):
      signed_fn = os.path.join(BASEDIR, "board", "obj", "panda.bin.signed")
      if os.path.exists(signed_fn):
        print("Flashing signed firmware")
        panda.flash(fn=signed_fn)
      else:
        print("Building and flashing unsigned firmware")
        panda.flash()

      print("Done flashing")

  if panda.bootstub:
    if panda.is_black() or (panda.health()['current'] > 2000):
      print("Flashed firmware not booting, flashing development bootloader")
      panda.recover()
      print("Done flashing bootloader")

  if panda.bootstub:
    if panda.is_black() or (panda.health()['current'] > 2000):
      print("Panda still not booting, exiting")
      raise AssertionError

  version = str(panda.get_version())
  if not version.startswith(repo_version):
    if panda.is_black() or (panda.health()['current'] > 2000):
      print("Version mismatch after flashing, exiting")
      raise AssertionError

def upload_drives():
  print("Attempting connection to panda")

  panda = None
  panda_list = Panda.list()
  if len(panda_list) == 0: 
    print("Panda disconnected, safe to upload")
    subprocess.call(['python3', 'upload_files.py'])


def main(gctx=None):
  setproctitle('pandad')
  try:
    kegman = kegman_conf()  #.read_config()
    if bool(int(kegman.conf['useAutoFlash'])): 
      update_panda()
    if bool(int(kegman.conf['autoUpload'])): 
      upload_drives()
    params = Params()
    panda = Panda.list()
    serial = Panda(panda[0]).get_serial()[0]
    print("Panda Serial: %s", serial, panda)
    if 'unprovisioned' in serial.decode('utf8'): serial = panda[0]
    params.put("PandaDongleId", serial)
  except:
    pass

  #update_panda()
  os.chdir("selfdrive/boardd")
  os.execvp("./boardd", ["./boardd"])

if __name__ == "__main__":
  main()
