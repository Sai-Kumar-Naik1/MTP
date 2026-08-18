"""Vehicle-link layer.

Everything that knows about MAVLink encoding lives here, so the GUI keeps
talking to the plain `contracts.gui_orchestration` schemas and never touches
the wire protocol itself (SRS §3.3).
"""
