"""Compatibility factory; synchronization decisions and events live in PnP.

New logs are compact communication CSV. Historical JSONL is read by
analyze_sync.py; there is no second observer state machine.
"""


def diagnostic_link(link_module, config, uart, clock, buffer_size=128):
    # Old arguments remain accepted; the shared link owns its event limit.
    return link_module.FlightLink(uart=uart, clock=clock)
