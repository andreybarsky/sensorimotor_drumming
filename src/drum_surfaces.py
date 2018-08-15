

drum_names = ["snare", "hihat", "cymbal1", "cymbal2", "tom1", "tom2", "bass"]

# (x,y,z, roll, pitch, yaw) in world coordinates: 
drum_poses = { 
    "snare": (0.798, -0.582, 0.922, 0,0,0),
    "hihat": (0.892, 0.531, 0.895, 0,0,0),
    "cymbal1": (0.932, 0.671, 1.404, -0.078, -0.041, 0.003),
    "cymbal2": (0.9567, -0.4134, 1.406, 0.058748, -0.067061, 0.002778),
    "tom1": (1.204, 0.260, 1.152, 0.260, -1.187, 0.176),
    "tom2": (1.201, -0.253, 1.147, -0.563, -1.187, 0.176),
    "bass": (1.051, 0.041, 0.390, 3.1416, -1.5708, 3.1416)
}

# cylinder radiuses:
# size_default = (0.05, 0.2)
drum_sizes = {
    "snare": 0.21,
    "hihat": 0.22,
    "cymbal1": 0.22,
    "cymbal2": 0.22,
    "tom1": 0.19,
    "tom2": 0.19,
    "bass": 0.35
}

drum_colours = {
    "snare": "green",
    "hihat": "red",
    "cymbal1": "yellow",
    "cymbal2": "orange",
    "tom1": "purple",
    "tom2": "teal",
    "bass": "blue"
}

colour_rgb = {
    "green": (0,1,0),
    "red": (1,0,0),
    "blue": (0,0.2,1),
    "yellow": (1,1,0),
    "orange": (1, 0.5, 0),
    "purple": (1, 0, 1),
    "teal": (0, 0.5, 1)
}

drum_soundfiles = {
    "snare": "samples/snare.wav",
    "hihat": "samples/hihat.wav",
    "cymbal1": "samples/crash.wav",
    "cymbal2": "samples/ride.wav",
    "tom1": "samples/tom1.wav",
    "tom2": "samples/tom2.wav",
    "bass": "samples/bass.wav"
}