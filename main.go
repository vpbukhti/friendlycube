package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/exec"
	"runtime"

	"friendlycube/internal/control"
	"friendlycube/internal/engine"
)

func main() {
	configPath := flag.String("config", "config.default.json", "path to a JSON config (absent keys / null → baked-in defaults)")
	seedFlag := flag.String("seed", "", "hex seed override (e.g. 1a2b3c). Empty = use config / random.")
	stlOnly := flag.String("stl", "", "if set, write STL to this path (headless) and exit")
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

	// Headless: build once at export resolution and write the STL.
	if *stlOnly != "" {
		seed, ok := control.ParseSeed(cfg.Seed)
		if !ok {
			seed = control.RandomSeed()
		}
		cfg.EnsureGeneratedSkeleton(seed)
		g := cfg.Build(seed, cfg.Skin.ExportResolution)
		if err := engine.WriteBinarySTL(*stlOnly, g); err != nil {
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
