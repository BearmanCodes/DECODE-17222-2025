{
  "startPoint": {
    "x": 128.5,
    "y": 111,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-8duf3dcitq4",
      "name": "Path 1",
      "endPoint": {
        "x": 88,
        "y": 87.5,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 46.5
      },
      "controlPoints": [],
      "color": "#A8DCB7",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle404ua-i1r99j",
      "name": "Path 2",
      "endPoint": {
        "x": 125,
        "y": 83.75,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 102,
          "y": 81.5
        }
      ],
      "color": "#8C999C",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle416bm-fxxyj6",
      "name": "Path 3",
      "endPoint": {
        "x": 88,
        "y": 87.5,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 46.5
      },
      "controlPoints": [],
      "color": "#57C99A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle428s0-y6eww5",
      "name": "Path 4",
      "endPoint": {
        "x": 115,
        "y": 83.75,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#978DDA",
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
      "lineId": "line-8duf3dcitq4"
    },
    {
      "kind": "path",
      "lineId": "mle404ua-i1r99j"
    },
    {
      "kind": "path",
      "lineId": "mle416bm-fxxyj6"
    },
    {
      "kind": "path",
      "lineId": "mle428s0-y6eww5"
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
  "timestamp": "2026-02-08T19:04:25.554Z"
}