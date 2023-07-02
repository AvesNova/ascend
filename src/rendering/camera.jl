
mutable struct Camera
    pose
end

function Camera()
    pos = rand(4)
    ori = rand(4)
    return Camera([pos; ori])
end