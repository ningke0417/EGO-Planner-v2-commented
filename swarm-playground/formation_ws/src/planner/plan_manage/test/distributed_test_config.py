"""Wire-format configuration fingerprints for isolated ROS test fixtures."""
import struct


def formation_digest(rows=2, columns=4, row_spacing=1.0, column_spacing=1.0):
    value = 14695981039346656037
    for byte in struct.pack('<QQdd', rows, columns, row_spacing, column_spacing):
        value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return value or 1
