{
  "targets": [
    {
      "target_name": "rpiopwm",
      "sources": [ "rpiopwm.cc", "pwm.c" ],
      "include_dirs": [
        "<!(node -e \"require('nan')\")"
      ]
    }
  ]
}