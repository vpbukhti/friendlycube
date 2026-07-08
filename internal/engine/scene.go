package engine

// Group is a tree node: either a Mesh leaf or a list of children, with a
// transform that composes onto its children.
type Group struct {
	Transform Transform
	Mesh      *Mesh
	Children  []*Group
}

func NewGroup() *Group {
	return &Group{Transform: IdentityTransform()}
}

func (g *Group) Add(child *Group) {
	if child == nil {
		return
	}
	g.Children = append(g.Children, child)
}

// AddMesh wraps a mesh as a leaf group with identity transform.
func (g *Group) AddMesh(m *Mesh) {
	leaf := NewGroup()
	leaf.Mesh = m
	g.Children = append(g.Children, leaf)
}

// Flatten walks the tree and emits triangles in world space, batched by
// material. Returns a slice of (material, triangles).
type MatBatch struct {
	Mat  Material
	Tris []Triangle
}

func (g *Group) Flatten() []MatBatch {
	bucket := make(map[matKey]int)
	var batches []MatBatch
	var walk func(node *Group, parent Transform)
	walk = func(node *Group, parent Transform) {
		t := Compose(parent, node.Transform)
		if node.Mesh != nil {
			k := matKey{node.Mesh.Mat.Color, node.Mesh.Mat.Emissive}
			idx, ok := bucket[k]
			if !ok {
				idx = len(batches)
				bucket[k] = idx
				batches = append(batches, MatBatch{Mat: node.Mesh.Mat})
			}
			transformMesh(node.Mesh, t, &batches[idx].Tris)
		}
		for _, c := range node.Children {
			walk(c, t)
		}
	}
	walk(g, IdentityTransform())
	return batches
}

type matKey struct {
	c [3]float32
	e bool
}

// addCylinder is a convenience: build a cylinder mesh placed with the given
// position + axis (axis is the direction from one cap to the other, with
// `height` along it).
func addCylinder(g *Group, mid, axis Vec3, height float64, rTop, rBot float64, radialSegs int, mat Material) {
	m := &Mesh{Tris: makeCylinder(rTop, rBot, height, radialSegs), Mat: mat}
	leaf := NewGroup()
	leaf.Mesh = m
	leaf.Transform.Position = mid
	leaf.Transform.Rotation = QuatFromUnitVectors(V(0, 1, 0), axis.Normalize())
	g.Children = append(g.Children, leaf)
}

func addSphere(g *Group, pos Vec3, radius float64, wSeg, hSeg int, mat Material) {
	m := &Mesh{Tris: makeSphere(radius, wSeg, hSeg), Mat: mat}
	leaf := NewGroup()
	leaf.Mesh = m
	leaf.Transform.Position = pos
	g.Children = append(g.Children, leaf)
}

func addOcta(g *Group, pos Vec3, radius float64, mat Material) {
	m := &Mesh{Tris: makeOctahedron(radius), Mat: mat}
	leaf := NewGroup()
	leaf.Mesh = m
	leaf.Transform.Position = pos
	g.Children = append(g.Children, leaf)
}
