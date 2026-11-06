package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/exec"
	"runtime"
	"sync/atomic"

	"friendlycube/internal/control"
)

func main() {
	configPath := flag.String("config", "config.default.json", "path to a JSON config (absent keys / null → baked-in defaults)")
	seedFlag := flag.String("seed", "", "hex seed override (e.g. 1a2b3c). Empty = use config / random.")
	stlOnly := flag.String("stl", "", "if set, write STL to this path (headless) and exit")
	splitOut := flag.String("split", "", "if set, split into printable pieces and write <path>.stl (all pieces) + <path>.tar (numbered pieces, rods, parts list), then exit")
	addr := flag.String("addr", "127.0.0.1:8730", "address for the control-panel + WebGL web server")
	open := flag.Bool("open", true, "open the panel in the default browser on start")
	flag.Parse()

	// Config: start from baked-in defaults, overlay the file if present.
	cfg := control.DefaultConfig()
	if *configPath != "" {
		if loaded, err := control.LoadConfig(*configPath); err != nil {
			if !os.IsNotExist(err) {
				log.Fatalf("config %s: %v", *configPath, err)
			}
			log.Printf("config %s not found — using baked-in defaults", *configPath)
		} else {
			cfg = loaded
		}
	}
	if *seedFlag != "" {
		cfg.Seed = *seedFlag
	}

	// Headless print split: plan, mesh every piece, write both files.
	if *splitOut != "" {
		seed, ok := control.ParseSeed(cfg.Seed)
		if !ok {
			seed = control.RandomSeed()
		}
		cfg.EnsureGeneratedSkeleton(seed)
		plan := cfg.PlanSplit(func(f float64) { fmt.Fprintf(os.Stderr, "\rplanning… %3d%%", int(f*100)) })
		fmt.Fprintln(os.Stderr)
		info := plan.Info()
		fmt.Printf("%d pieces, %d cuts, rods: %d main + %d thin, %d dash keys, %d pieces need support\n",
			len(info.Pieces), len(info.Cuts), info.RodsMain, info.RodsThin, info.DashKeys, info.Flagged)
		err := plan.ExportSplit(*splitOut+".stl", *splitOut+".tar", cfg.Print.VoxelMM,
			func(f float64) { fmt.Fprintf(os.Stderr, "\rmeshing pieces… %3d%%", int(f*100)) })
		fmt.Fprintln(os.Stderr)
		if err != nil {
			log.Fatalf("split: %v", err)
		}
		fmt.Printf("wrote %s.stl and %s.tar (seed=%06x, cell=%.2f mm)\n", *splitOut, *splitOut, seed, cfg.Print.VoxelMM)
		return
	}

	// Headless: build once at export resolution and write the STL.
	if *stlOnly != "" {
		seed, ok := control.ParseSeed(cfg.Seed)
		if !ok {
			seed = control.RandomSeed()
		}
		cfg.EnsureGeneratedSkeleton(seed)
		var lastPct atomic.Int64
		lastPct.Store(-1)
		progress := func(f float64) {
			pct := int64(f * 100)
			for {
				old := lastPct.Load()
				if pct <= old {
					return
				}
				if lastPct.CompareAndSwap(old, pct) {
					fmt.Fprintf(os.Stderr, "\rmeshing… %3d%%", pct)
					return
				}
			}
		}
		err := cfg.StreamSTL(*stlOnly, seed, cfg.Skin.ExportResolution, progress)
		fmt.Fprintln(os.Stderr)
		if err != nil {
			log.Fatalf("stl: %v", err)
		}
		fmt.Printf("wrote %s (seed=%06x, res=%d)\n", *stlOnly, seed, cfg.Skin.ExportResolution)
		return
	}

	app := control.NewApp(cfg, *configPath)
	app.Start() // background build worker + first build

	url := "http://" + *addr
	fmt.Printf("friendlycube — open %s\n", url)
	if *open {
		go openBrowser(url)
	}
	if err := app.Serve(*addr); err != nil {
		log.Fatalf("server: %v", err)
	}
}

// openBrowser best-effort launches the OS default browser. Errors are ignored;
// the URL is always printed so the user can open it manually.
func openBrowser(url string) {
	var cmd string
	var args []string
	switch runtime.GOOS {
	case "darwin":
		cmd = "open"
	case "windows":
		cmd, args = "rundll32", []string{"url.dll,FileProtocolHandler"}
	default:
		cmd = "xdg-open"
	}
	_ = exec.Command(cmd, append(args, url)...).Start()
}
