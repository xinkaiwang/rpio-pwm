{
  "targets": [
    {
      "target_name": "rpiopwm",
      "sources": [ "rpiopwm.cc", "pwm.cpp", "mailbox.c", "dma.cpp" ],
      "include_dirs": [
        "<!(node -e \"require('nan')\")",
        "/opt/vc/include"
      ],
      "libraries": [
        "-lbcm_host",
        "-L/opt/vc/lib"
      ]
    }
  ]
}