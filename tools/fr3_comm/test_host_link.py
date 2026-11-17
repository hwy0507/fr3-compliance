"""Offline driver-readback regressions. No socket or real NIC setter is used."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import host_link as link


class EeeReadbackTests(unittest.TestCase):
    def setUp(self):
        self.original = dict(zip(link.FIELDS, [0, 40, 40, 40, 1, 1, 1, 12, 0, 0]))
        self.disabled = dict(self.original, advertised=0, eee_active=0,
                             eee_enabled=0, tx_lpi_enabled=0)

    def test_disabled_advertisement_zero_is_valid(self):
        link.validate_readback(self.disabled, 0, 0, 40, 12)

    def test_disabled_advertisement_retained_is_valid(self):
        link.validate_readback(dict(self.disabled, advertised=40), 0, 0, 40, 12)

    def test_enabled_original_is_valid(self):
        link.validate_readback(self.original, 1, 1, 40, 12)

    def test_restore_negotiation_not_yet_active_is_valid(self):
        link.validate_readback(dict(self.original, eee_active=0), 1, 1, 40, 12)

    def test_disable_failures_are_rejected(self):
        for key, value in [('eee_enabled', 1), ('tx_lpi_enabled', 1),
                           ('eee_active', 1), ('advertised', 8), ('tx_lpi_timer', 99)]:
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                link.validate_readback(dict(self.disabled, **{key: value}), 0, 0, 40, 12)

    def test_restore_must_restore_advertisement(self):
        with self.assertRaises(RuntimeError):
            link.validate_readback(dict(self.original, advertised=0), 1, 1, 40, 12)

    def test_setter_preserves_original_payload_and_checks_actual_result(self):
        with patch.object(link, 'ioctl_eee') as ioctl, patch.object(link, 'read_eee', return_value=self.disabled):
            result = link.set_eee(self.original, 0, 0, 40, 12)
        self.assertEqual(result, self.disabled)
        self.assertEqual(ioctl.call_args.args[0], [link.SEEE, 40, 40, 40, 1, 0, 0, 12, 0, 0])
        self.assertEqual(self.original['advertised'], 40)

    def test_existing_backup_never_overwritten_and_setter_not_called(self):
        identity = {'interface': link.NIC, 'pci': link.PCI, 'mac': 'test'}
        with tempfile.TemporaryDirectory() as temp:
            backup = Path(temp) / 'original.json'
            saved = json.dumps({'preserve': True}); backup.write_text(saved)
            with patch('sys.argv', ['host_link', '--disable', str(backup)]), \
                 patch.object(link, 'identity', return_value=identity), \
                 patch.object(link, 'read_eee', return_value=self.disabled), \
                 patch.object(link.os, 'geteuid', return_value=0), \
                 patch.object(link, 'set_eee') as setter:
                with self.assertRaises(FileExistsError):
                    link.main()
                setter.assert_not_called()
            self.assertEqual(backup.read_text(), saved)


if __name__ == '__main__':
    unittest.main()
