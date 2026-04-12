from __future__ import annotations

import tkinter as tk
from typing import Callable


def place_window_over_parent(
    window: tk.Toplevel,
    parent: tk.Widget,
    *,
    cascade_px: int = 0,
) -> None:
    try:
        parent.update_idletasks()
        window.update_idletasks()
    except Exception:
        return

    parent_x = int(parent.winfo_rootx())
    parent_y = int(parent.winfo_rooty())
    parent_w = max(1, int(parent.winfo_width()))
    parent_h = max(1, int(parent.winfo_height()))
    width = max(1, int(window.winfo_width() or window.winfo_reqwidth()))
    height = max(1, int(window.winfo_height() or window.winfo_reqheight()))

    x = parent_x + max(0, (parent_w - width) // 2) + int(cascade_px)
    y = parent_y + max(0, (parent_h - height) // 2) + int(cascade_px)
    window.geometry(f"+{x}+{y}")


def prepare_toplevel(window: tk.Toplevel, parent: tk.Widget, *, cascade_px: int = 0) -> None:
    try:
        window.transient(parent)
    except Exception:
        pass
    place_window_over_parent(window, parent, cascade_px=cascade_px)
    try:
        window.lift(parent)
    except Exception:
        pass
    try:
        window.focus_set()
    except Exception:
        pass


def bind_vertical_mousewheel(widget: tk.Widget, *, target: tk.Misc | None = None) -> Callable[[tk.Event], str]:
    scroll_target = target or widget

    def _scroll(event: tk.Event) -> str:
        if getattr(event, "num", None) == 4:
            delta = -1
        elif getattr(event, "num", None) == 5:
            delta = 1
        else:
            delta_value = getattr(event, "delta", 0)
            delta = -1 if delta_value > 0 else 1
        try:
            scroll_target.yview_scroll(delta, "units")
        except Exception:
            return "break"
        return "break"

    for sequence in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
        widget.bind(sequence, _scroll, add="+")
    return _scroll


def bind_vertical_mousewheel_tree(widget: tk.Widget, *, target: tk.Misc | None = None) -> None:
    bind_vertical_mousewheel(widget, target=target)
    for child in widget.winfo_children():
        bind_vertical_mousewheel_tree(child, target=target)
