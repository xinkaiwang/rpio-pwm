{
  "targets": [
    {
      "target_name": "rpiopwm",
      "sources": [ "rpiopwm.cc", "pwm.c", "mailbox.c" ],
      "include_dirs": [
        "<!(node -e \"require('nan')\")"
      ]
    }
  ]
}