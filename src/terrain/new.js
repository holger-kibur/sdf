var BaryMeshVertex = function(vert_list_ref, vert_index) {
    this.vert_list_ref = vert_list_ref;
    this.vert_index = vert_index;
};

BaryMeshVertex.prototype.x = function() {
    return this.vert_list_ref[this.vert_index * 3];
};

BaryMeshVertex.prototype.y = function() {
    return this.vert_list_ref[this.vert_index * 3 + 1];
};

BaryMeshVertex.prototype.z = function() {
    return this.vert_list_ref[this.vert_index * 3 + 2];
};

BaryMeshVertex.prototype.distance_squared = function(other_vertex) {
    var dx = other_vertex.x() - this.x();
    var dy = other_vertex.y() - this.y();
    var dz = other_vertex.z() - this.z();
    return dx * dx + dy * dy + dz * dz;
};

BaryMeshVertex.prototype.get_global_point = function(global_a, global_b, global_c) {
    return new BaryFakeVertex(
        global_a.x() * this.x() + global_b.x() * this.y() + global_c.x() * this.z(),
        global_a.y() * this.x() + global_b.y() * this.y() + global_c.y() * this.z(),
        global_a.z() * this.x() + global_b.z() * this.y() + global_c.z() * this.z()
    );
};

var BaryFakeVertex = function(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
};

BaryFakeVertex.prototype.x = function() {
    return this.x;
};

BaryFakeVertex.prototype.y = function() {
    return this.y;
};

BaryFakeVertex.prototype.z = function() {
    return this.z;
};

BaryFakeVertex.prototype.distance_squared = BaryMeshVertex.prototype.distance_squared;
BaryFakeVertex.prototype.get_global_point = BaryMeshVertex.prototype.get_global_point;

var Edge = function(from_vertex, to_vertex) {
    this.from_vertex = from_vertex;
    this.to_vertex = to_vertex;
};

Edge.prototype.get_length_squared = function() {
    return this.from_vertex.distance_squared(this.to_vertex);
};

var Triangle = function(edge_ab, edge_bc, edge_ca) {
    this.edge_ab = edge_ab;
    this.edge_bc = edge_bc;
    this.edge_ca = edge_ca;

    var ab_length_squared = edge_ab.get_length_squared();
    var bc_length_squared = edge_bc.get_length_squared();
    var ca_length_squared = edge_ca.get_length_squared();
    var local_circum_x = bc_length_squared * (ca_length_squared + ab_length_squared - bc_length_squared);
    var local_circum_y = ca_length_squared * (ab_length_squared + bc_length_squared - ca_length_squared);
    var local_circum_z = ab_length_squared * (bc_length_squared + ca_length_squared - ab_length_squared);
    var local_mag = local_circum_x + local_circum_y + local_circum_z;
    local_circum_x /= local_mag;
    local_circum_y /= local_mag;
    local_circum_z /= local_mag;
    var local_point = new BaryFakeVertex(local_circum_x, local_circum_y, local_circum_z);
    
    this.circum_point = local_point.get_global_point(this.edge_ab.from_vertex, this.edge_bc.from_vertex, this.edge_ca.from_vertex);
    this.circum_radius_squared = this.circum_point.distance_squared(this.edge_ab.from_vertex);
};

var FractalTerrainGenerator = function() {
    this.vertex_list = [];
    this.tesselation_queue = [];
};

FractalTerrainGenerator.prototype.get_vertex