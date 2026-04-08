"""Backward-compatible MainWindow import and launcher."""

from simulation_app.ui.main_window import MainWindow, main

__all__ = ["MainWindow", "main"]


if __name__ == "__main__":
    main()
