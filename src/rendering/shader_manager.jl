import GLAbstraction as GLA

function get_vertex_shader(vertex_dir:: String = "./assets/shaders/vertex/vertex.glsl")
    vertex_source::String = read(vertex_dir, String)
    vertex_shader = GLA.Shader(GL_VERTEX_SHADER, vertex_source)
    return vertex_shader
end

function get_fragment_shader(fragment_dir:: String = "./assets/shaders/fragment/fragment.glsl")
    fragment_source::String = read(fragment_dir, String)
    fragment_shader = GLA.Shader(GL_FRAGMENT_SHADER, fragment_source)
    return fragment_shader
end
