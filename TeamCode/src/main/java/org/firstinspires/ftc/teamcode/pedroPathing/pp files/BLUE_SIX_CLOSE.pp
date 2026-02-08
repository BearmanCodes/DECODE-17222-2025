{
  "startPoint": {
    "x": 15.5,
    "y": 111,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-ibaxwbixnz",
      "name": "Path 1",
      "endPoint": {
        "x": 63.5,
        "y": 83.5,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 139
      },
      "controlPoints": [],
      "color": "#7C87DC",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle45g4a-93lxxh",
      "name": "Path 2",
      "endPoint": {
        "x": 25,
        "y": 84.5,
        "heading": "constant",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#57DCDC",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle46mj3-j6bb72",
      "name": "Path 3",
      "endPoint": {
        "x": 63.5,
        "y": 83.5,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 139
      },
      "controlPoints": [],
      "color": "#B9CB9A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle470vj-m0ppmz",
      "name": "Path 4",
      "endPoint": {
        "x": 19,
        "y": 84.5,
        "heading": "constant",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#7A7BBD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-ibaxwbixnz"
    },
    {
      "kind": "path",
      "lineId": "mle45g4a-93lxxh"
    },
    {
      "kind": "path",
      "lineId": "mle46mj3-j6bb72"
    },
    {
      "kind": "path",
      "lineId": "mle470vj-m0ppmz"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 18,
    "rHeight": 18,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "dark",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#F5A9B8",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-02-08T19:07:43.382Z"
}