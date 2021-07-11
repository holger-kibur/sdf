var SQRT_3 = sqrt(3);

var CLOSE_RESOLUTION = 5;
var RENDER_DISTANCE = 100;
var CR_OVER_RD = CLOSE_RESOLUTION / RENDER_DISTANCE;

var hash = function(x) {
    // From murmer 
    x ^= x >> 16;
    x *= 0x85EBCA6B;
    x ^= x >> 13;
    x *= 0xC2B2AE35;
    x ^= x >> 16;
    return x;
};

var circum_determinate = function(a, b, c, p){
    var dax = a[0] - p[0];
    var day = a[1] - p[1];
    var dbx = b[0] - p[0];
    var dby = b[1] - p[1];
    var dcx = c[0] - p[0];
    var dcy = c[1] - p[1];
    return (dax*dax + day*day) * (dbx*dcy - dcx*dby) - (dbx*dbx + dby*dby) * (dax*dcy - dcx*day) + (dcx*dcx + dcy*dcy) * (dax*dby - dbx*day);
};

var fractal_func = function(d_squared) {
    var numer = CR_OVER_RD * (RENDER_DISTANCE + 1);
    var denom = d_squared / RENDER_DISTANCE + 1;
    return floor(numer / denom - CR_OVER_RD) + 1;
};

var tri_from_incircle = function(x, y, r) {
    return [
        x,            y + 2*r,
        x - r*SQRT_3, y - r,
        x + r*SQRT_3, y - r,
    ];
};

var MeshEdge = function(v1_index, v2_index) {
    this.v1_index = v1_index;
    this.v2_index = v2_index;
};

var MeshTri = function(edge_ab, edge_bc, edge_ca, ca, cb, cc, cr_sq) {
    this.edge_ab = edge_ab;
    this.edge_bc = edge_bc;
    this.edge_ca = edge_ca;
    this.ca = ca;
    this.cb = cb;
    this.cc = cc;
    this.cr_sq = cr_sq;
};

MeshTri.prototype.get_a_index = function(){
    return this.edge_ab.v1_index;
};

MeshTri.prototype.get_b_index = function(){
    return this.edge_ab.v2_index;
};

MeshTri.prototype.get_c_index = function(){
    return this.edge_bc.v2_index;
};

var MeshBuilder = function(a, b, c) {
    // One vertex = x, y
    this.super_ax = a[0];
    this.super_ay = a[1];
    this.super_bx = b[0];
    this.super_by = b[1];
    this.super_cx = c[0];
    this.super_cy = c[1];
    
    this.verts = [
        1, 0, 0, // a
        0, 1, 0, // b
        0, 0, 1, // c
    ];
    
    // One edge = two connected verticies
    this.edges = [
        new MeshEdge(0, 1),
        new MeshEdge(1, 2),
        new MeshEdge(2, 0),
    ];
    
    // One triangle = three edges
    this.triangles = [
        this.triangle_from_edges(this.edges[0], this.edges[1], this.edges[2]),
    ];
};

MeshBuilder.prototype.get_cart_x = function(a, b, c) {
    return a * this.super_ax + b * this.super_bx + c * this.super_cx;
};

MeshBuilder.prototype.get_cart_y = function(a, b, c) {
    return a * this.super_ay + b * this.super_by + c * this.super_cy;
};

MeshBuilder.prototype.gva = function(vert_ind) {
    return this.verts[vert_ind * 3];
};

MeshBuilder.prototype.gvb = function(vert_ind) {
    return this.verts[vert_ind * 3 + 1];
};

MeshBuilder.prototype.gvc = function(vert_ind) {
    return this.verts[vert_ind * 3 + 2];
};

MeshBuilder.prototype.get_cart_dist_sq_raw = function(a1, b1, c1, a2, b2, c2) {
    var v1x = this.get_cart_x(a1, b1, c1);
    var v1y = this.get_cart_y(a1, b1, c1);
    var v2x = this.get_cart_x(a2, b2, c2);
    var v2y = this.get_cart_y(a2, b2, c2);
    var dx = v1x - v2x;
    var dy = v1y - v2y;
    return dx * dx + dy * dy;
};

MeshBuilder.prototype.get_cart_dist_sq_half = function(v1_ind, a, b, c) {
    return this.get_cart_dist_sq_raw(
        this.verts[v1_ind * 3],
        this.verts[v1_ind * 3 + 1],
        this.verts[v1_ind * 3 + 2],
        a,
        b,
        c
    );
};

MeshBuilder.prototype.get_cart_dist_sq = function(v1_ind, v2_ind){
    return this.get_cart_dist_sq_half(
        v1_ind,
        this.verts[v2_ind * 3],
        this.verts[v2_ind * 3 + 1],
        this.verts[v2_ind * 3 + 2]
    );
};

MeshBuilder.prototype.triangle_from_edges = function(edge_ab, edge_bc, edge_ca){
    var a_sq = this.get_cart_dist_sq(edge_bc.v1_index, edge_bc.v2_index);
    var b_sq = this.get_cart_dist_sq(edge_ca.v1_index, edge_ca.v2_index);
    var c_sq = this.get_cart_dist_sq(edge_ab.v1_index, edge_ab.v2_index);
    var lca = a_sq * (b_sq + c_sq - a_sq);
    var lcb = b_sq * (c_sq + a_sq - b_sq);
    var lcc = c_sq * (a_sq + b_sq - c_sq);
    var m = (lca + lcb + lcc);
    lca /= m;
    lcb /= m;
    lcc /= m;
    var ca = this.gva(edge_ab.v1_index) * lca + this.gva(edge_ab.v2_index) * lcb + this.gva(edge_bc.v2_index) * lcc;
    var cb = this.gvb(edge_ab.v1_index) * lca + this.gvb(edge_ab.v2_index) * lcb + this.gvb(edge_bc.v2_index) * lcc;
    var cc = this.gvc(edge_ab.v1_index) * lca + this.gvc(edge_ab.v2_index) * lcb + this.gvc(edge_bc.v2_index) * lcc;
    var cr_sq = this.get_cart_dist_sq_half(edge_ab.v1_index, ca, cb, cc);
    return new MeshTri(edge_ab, edge_bc, edge_ca, ca, cb, cc, cr_sq);
};

MeshBuilder.prototype.find_edge = function(edge) {
    for (var i = 0; i < this.edges.length; i++) {
        if (this.edges[i] === edge) {
            return i;
        }
    }
    return -1;
};

MeshBuilder.prototype.insert_vert = function(a, b, c) {
    var bad_tris = [];
    var touched_edges = Array(this.edges.length);
    touched_edges.fill(0);
    var new_vert_index = this.verts.length / 3;
    this.verts.push(a, b, c);
    for (var i = 0; i < this.triangles.length; i += 1) {
        var tri = this.triangles[i];
        if (this.get_cart_dist_sq_raw(a, b, c, tri.ca, tri.cb, tri.cc) < tri.cr_sq) {
            bad_tris.push(i);
        }
    }
    for (var i = 0; i < bad_tris.length; i++) {
        var tri = this.triangles[bad_tris[i]];
        touched_edges[this.find_edge(tri.edge_ab)]++;
        touched_edges[this.find_edge(tri.edge_bc)]++;
        touched_edges[this.find_edge(tri.edge_ca)]++;
    }
    for (var i = bad_tris.length - 1; i >= 0; i--) {
        this.triangles.splice(bad_tris[i], 1);
    }
    var new_edges = [];
    for (var i = 0; i < touched_edges.length; i++) {
        if (touched_edges[i] !== 1) {
            continue;
        }
        var poly_edge = this.edges[i];
        var v1_edge = null;
        var v2_edge = null;
        for (var j = 0; j < new_edges.length; j++) {
            if (new_edges[j].v1_index === poly_edge.v1_index) {
                v1_edge = new_edges[j];
            } else if (new_edges[j].v1_index === poly_edge.v2_index) {
                v2_edge = new_edges[j];
            }
        }
        if (v1_edge === null) {
            v1_edge = new MeshEdge(poly_edge.v1_index, new_vert_index);
            new_edges.push(v1_edge);
            this.edges.push(v1_edge);
        }
        if (v2_edge === null) {
            v2_edge = new MeshEdge(poly_edge.v2_index, new_vert_index);
            new_edges.push(v2_edge);
            this.edges.push(v2_edge);
        }
        this.triangles.push(this.triangle_from_edges(poly_edge, v2_edge, v1_edge));
    }
    for (var i = this.edges.length - 1; i >= 0; i--) {
        if (i < touched_edges.length && touched_edges[i] > 1) {
            this.edges.splice(i, 1);
        }
    }
};

MeshBuilder.prototype.show_edges = function() {
    for (var i = 0; i < this.edges.length; i++) {
        var v1_idx = this.edges[i].v1_index;
        var v2_idx = this.edges[i].v2_index;
        if (v1_idx - 3 < 0 || v2_idx - 3 < 0) {
            continue;
        }
        var v1x = this.get_cart_x(this.verts[v1_idx * 3], this.verts[v1_idx * 3 + 1], this.verts[v1_idx * 3 + 2]);
        var v1y = this.get_cart_y(this.verts[v1_idx * 3], this.verts[v1_idx * 3 + 1], this.verts[v1_idx * 3 + 2]);
        var v2x = this.get_cart_x(this.verts[v2_idx * 3], this.verts[v2_idx * 3 + 1], this.verts[v2_idx * 3 + 2]);
        var v2y = this.get_cart_y(this.verts[v2_idx * 3], this.verts[v2_idx * 3 + 1], this.verts[v2_idx * 3 + 2]);
        line(v1x, v1y, v2x, v2y);
    }
};

MeshBuilder.prototype.label_triangles = function() {
    for (var i = 0; i < this.triangles.length; i++) {
        var tri = this.triangles[i];
        var v1_idx = tri.get_a_index();
        var v2_idx = tri.get_b_index();
        var v3_idx = tri.get_c_index();
        var v1x = this.get_cart_x(this.verts[v1_idx * 3], this.verts[v1_idx * 3 + 1], this.verts[v1_idx * 3 + 2]);
        var v1y = this.get_cart_y(this.verts[v1_idx * 3], this.verts[v1_idx * 3 + 1], this.verts[v1_idx * 3 + 2]);
        var v2x = this.get_cart_x(this.verts[v2_idx * 3], this.verts[v2_idx * 3 + 1], this.verts[v2_idx * 3 + 2]);
        var v2y = this.get_cart_y(this.verts[v2_idx * 3], this.verts[v2_idx * 3 + 1], this.verts[v2_idx * 3 + 2]);
        var v3x = this.get_cart_x(this.verts[v3_idx * 3], this.verts[v3_idx * 3 + 1], this.verts[v3_idx * 3 + 2]);
        var v3y = this.get_cart_y(this.verts[v3_idx * 3], this.verts[v3_idx * 3 + 1], this.verts[v3_idx * 3 + 2]);
        var avg_x = (v1x + v2x + v3x) / 3.0;
        var avg_y = (v1y + v2y + v3y) / 3.0;
        text("tri " + i, avg_x, avg_y);
        var circ_x = this.get_cart_x(tri.ca, tri.cb, tri.cc);
        var circ_y = this.get_cart_y(tri.ca, tri.cb, tri.cc);
        ellipse(circ_x, circ_y, 5, 5);
        text("tri " + i, circ_x, circ_y);
    }
};

MeshBuilder.prototype.insert_from_cart = function(x, y) {
    var det = (this.super_by - this.super_cy) * (this.super_ax - this.super_cx) + (this.super_cx - this.super_bx) * (this.super_ay - this.super_cy);
    var a = ((this.super_by - this.super_cy) * (x - this.super_cx) + (this.super_cx - this.super_bx) * (y - this.super_cy)) / det;
    var b = ((this.super_cy - this.super_ay) * (x - this.super_cx) + (this.super_ax - this.super_cx) * (y - this.super_cy)) / det;
    var c = 1 - a - b;
    this.insert_vert(a, b, c);
};

var player_pos = [187, 200];
var verts = [];

fill(255, 0, 0);

var builder = new MeshBuilder([272, -68], [-1, 358], [400, 400]);
for (var i = 0; i < 100; i++) {
    var u1_sqrt = sqrt(random(0, 1));
    var u2 = random(0, 1);
    builder.insert_vert(1 - u1_sqrt, u1_sqrt * (1 - u2), u1_sqrt * u2);
}
ellipse(168, 282, 5, 5);
builder.show_edges();

