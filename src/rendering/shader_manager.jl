using ModernGL
import GLAbstraction as GLA

"""
get_vertex_shader(vertex_dir::String="./assets/shaders/vertex/vertex.glsl") -> Shader

Read the vertex shader file from the given directory and return a `Shader` object.

# Arguments
- `vertex_dir::String`: The directory of the vertex shader file. Defaults to "./assets/shaders/vertex/vertex.glsl".

# Returns
- `Shader`: The vertex shader object.
"""
function get_vertex_shader(vertex_dir::String = "./assets/shaders/vertex/vertex.glsl")::GLA.Shader
    vertex_source::String = read(vertex_dir, String)
    vertex_shader = GLA.Shader(GL_VERTEX_SHADER, vertex_source)
    return vertex_shader
end


"""
    get_fragment_shader(fragment_dir::String="./assets/shaders/fragment/fragment.glsl") -> Shader

Read the fragment shader file from the given directory and return a `Shader` object.

# Arguments
- `fragment_dir::String`: The directory of the fragment shader file. Defaults to "./assets/shaders/fragment/fragment.glsl".

# Returns
- `Shader`: The fragment shader object.
"""
function get_fragment_shader(fragment_dir::String = "./assets/shaders/fragment/fragment.glsl")::GLA.Shader
    fragment_source::String = read(fragment_dir, String)
    fragment_shader = GLA.Shader(GL_FRAGMENT_SHADER, fragment_source)
    return fragment_shader
end
