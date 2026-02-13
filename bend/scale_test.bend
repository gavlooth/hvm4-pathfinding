# Scale test - simple count

def count(n, acc):
  if n == 0:
    return acc
  else:
    return count(n - 1, acc + 1)

def main():
  return count(10000, 0)
