package engine

// StreamSkinSTL meshes the skin for the given skeleton and writes it straight to
// an STL file, slab by slab, without ever holding the full sample grid or the
// full triangle set in memory. Peak memory is O(res²) for the grid window plus
// one slab of triangles — so even very high export resolutions stay bounded.
//
// overlay (e.g. the handshake arms) is optional; its triangles are appended
// after the skin, matching the order WriteBinarySTL would produce for a scene
// with the skin first and the overlay second.
//
// The relax pass needs the whole mesh at once, so when sp.Relax > 0 this falls
// back to the in-memory build + WriteBinarySTL (relax is off by default).
func StreamSkinSTL(path string, s Settings, sk *Skeleton, sp SkinParams, overlay *Group) error {
	if sp.Relax > 0 {
		g := BuildSkinFromSkeleton(s, sk, sp)
		if overlay != nil {
			g.Add(overlay)
		}
		return WriteBinarySTL(path, g)
	}

	field, box, _, _ := skinField(s, sk, sp)

	sink, err := newSTLSink(path)
	if err != nil {
		return err
	}
	marchStream(field, box, sp.Resolution, sp.Progress, sink.writeTris)
	if overlay != nil {
		for _, b := range overlay.Flatten() {
			sink.writeTris(b.Tris)
		}
	}
	return sink.Close()
}
