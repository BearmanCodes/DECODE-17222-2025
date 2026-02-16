{
  "startPoint": {
    "x": 55.92558139534884,
    "y": 8.037209302325575,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-wkb7dd4ppkk",
      "name": "Path 1",
      "endPoint": {
        "x": 61.5,
        "y": 12,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 111
      },
      "controlPoints": [],
      "color": "#DA7879",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3kpoe-miqj78",
      "name": "Path 2",
      "endPoint": {
        "x": 11,
        "y": 35,
        "heading": "constant",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 67.42,
          "y": 39.12
        }
      ],
      "color": "#DB5BB6",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3ly4p-9bko8c",
      "name": "Path 3",
      "endPoint": {
        "x": 61.5,
        "y": 14,
        "heading": "linear",
        "reverse": false,
        "endDeg": 115,
        "startDeg": 0
      },
      "controlPoints": [],
      "color": "#BD65CB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3mr1j-jylo64",
      "name": "Path 4",
      "endPoint": {
        "x": 11,
        "y": 57.25,
        "heading": "constant",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 67.42,
          "y": 65.5
        }
      ],
      "color": "#6D6B8B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3nn0i-1pdr04",
      "name": "Path 5",
      "endPoint": {
        "x": 61.5,
        "y": 11.5,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 115
      },
      "controlPoints": [],
      "color": "#789C69",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mle3ogfu-lfriud",
      "name": "Path 6",
      "endPoint": {
        "x": 30,
        "y": 11.5,
        "heading": "linear",
        "reverse": false,
        "startDeg": 115,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#B8AB66",
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
      "lineId": "line-wkb7dd4ppkk"
    },
    {
      "kind": "path",
      "lineId": "mle3kpoe-miqj78"
    },
    {
      "kind": "path",
      "lineId": "mle3ly4p-9bko8c"
    },
    {
      "kind": "path",
      "lineId": "mle3mr1j-jylo64"
    },
    {
      "kind": "path",
      "lineId": "mle3nn0i-1pdr04"
    },
    {
      "kind": "path",
      "lineId": "mle3ogfu-lfriud"
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
  "timestamp": "2026-02-08T18:53:50.114Z"
}