"""Graphical user interface launcher for the simulation.

This module provides backward-compatible import and launcher for the
Heterogeneous Network Simulation graphical user interface.

It serves as the main entry point for starting the UI application.

Usage:
    Command line:
        python ground_main.py
    
    After installation via pip:
        simulation-ui

The module re-exports the MainWindow class and main function from
simulation_app.ui.main_window for backward compatibility with older code.
"""

from simulation_app.ui.main_window import MainWindow, main

__all__ = ["MainWindow", "main"]


if __name__ == "__main__":
    main()
