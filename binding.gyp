{
  "targets": [
    {
      "target_name": "pwm",
      "sources": [ "pwm.cc" ],
      "include_dirs": [
        "<!(node -e \"require('nan')\")"
      ]
    }
  ]
}