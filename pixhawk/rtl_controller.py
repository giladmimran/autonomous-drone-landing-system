def dry_run(status):
    missing = [k for k in ('connected', 'gps', 'home', 'rtl_supported', 'state_approved') if not status.get(k)]
    if missing:
        return 'DRY-RUN RTL blocked: ' + ', '.join(missing)
    return 'DRY-RUN: RTL request WOULD be issued to configured home; no command sent'
