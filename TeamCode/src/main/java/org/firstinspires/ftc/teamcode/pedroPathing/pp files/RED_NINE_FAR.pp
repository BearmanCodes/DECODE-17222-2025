{
  "startPoint": {
    "x": 88.0744186,
    "y": 8.037209302325575,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-ahgzq1lec9k",
      "name": "Path 1",
      "endPoint": {
        "x": 84.5,
        "y": 12,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 69
      },
      "controlPoints": [],
      "color": "#CCCDDD",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3tii6-okhei9",
      "name": "Path 2",
      "endPoint": {
        "x": 132.75,
        "y": 40,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 73.059026559147,
          "y": 44
        }
      ],
      "color": "#98A97A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3u4ku-sxgyj6",
      "name": "Path 3",
      "endPoint": {
        "x": 86.5,
        "y": 15,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 67
      },
      "controlPoints": [],
      "color": "#6D787A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3uj4s-gg65c6",
      "name": "Path 4",
      "endPoint": {
        "x": 132.5,
        "y": 63,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 71.75,
          "y": 67
        }
      ],
      "color": "#BAC8B6",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3v5lv-zuvgvz",
      "name": "Path 5",
      "endPoint": {
        "x": 86.5,
        "y": 15,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 67
      },
      "controlPoints": [],
      "color": "#B776B9",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3vq6l-8sc8ox",
      "name": "Path 6",
      "endPoint": {
        "x": 115,
        "y": 11.5,
        "heading": "linear",
        "reverse": false,
        "startDeg": 67,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#C68ABD",
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
      "lineId": "line-ahgzq1lec9k"
    },
    {
      "kind": "path",
      "lineId": "mle3tii6-okhei9"
    },
    {
      "kind": "path",
      "lineId": "mle3u4ku-sxgyj6"
    },
    {
      "kind": "path",
      "lineId": "mle3uj4s-gg65c6"
    },
    {
      "kind": "path",
      "lineId": "mle3v5lv-zuvgvz"
    },
    {
      "kind": "path",
      "lineId": "mle3vq6l-8sc8ox"
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
  "timestamp": "2026-02-08T18:59:03.713Z"
}