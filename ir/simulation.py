from .morse_decoder import TABLE

def simulate(message, unit, repetitions=2):
    t = 0.0
    yield t, False, 1.0
    t += 11 * unit
    for _ in range(repetitions):
        for letter in message:
            for j, symbol in enumerate(TABLE[letter]):
                yield t, True, 1.0
                t += unit * (1 if symbol == '.' else 3)
                yield t, False, 1.0
                t += unit if j < len(TABLE[letter]) - 1 else 3 * unit
        t += 8 * unit
        yield t, False, 1.0
        t += unit
