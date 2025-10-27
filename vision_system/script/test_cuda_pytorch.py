#!/usr/bin/env python3
import torch
import time

print("===== 🔍 PyTorch + CUDA Diagnostic Script =====\n")

# Version info
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available:  {torch.cuda.is_available()}")

if torch.cuda.is_available():
    print(f"CUDA device count: {torch.cuda.device_count()}")
    for i in range(torch.cuda.device_count()):
        print(f"  ▶ Device {i}: {torch.cuda.get_device_name(i)}")
    print(f"Current device: {torch.cuda.current_device()}\n")
else:
    print("⚠️  CUDA non disponibile — verrà usata solo la CPU.\n")

# Simple tensor test
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# Test computation speed
size = 10000
print(f"\nRunning matrix multiplication benchmark ({size}x{size})...\n")

a_cpu = torch.randn(size, size)
b_cpu = torch.randn(size, size)

# CPU timing
start_cpu = time.time()
c_cpu = torch.mm(a_cpu, b_cpu)
cpu_time = time.time() - start_cpu
print(f"⏱️  CPU time: {cpu_time:.3f} s")

if torch.cuda.is_available():
    a_gpu = a_cpu.to(device)
    b_gpu = b_cpu.to(device)
    torch.cuda.synchronize()
    start_gpu = time.time()
    c_gpu = torch.mm(a_gpu, b_gpu)
    torch.cuda.synchronize()
    gpu_time = time.time() - start_gpu
    print(f"⚡ GPU time: {gpu_time:.3f} s")
    print(f"💪 Speedup:  {cpu_time / gpu_time:.1f}x faster on GPU\n")
else:
    print("\n💤 Solo CPU disponibile.\n")

print("===== ✅ Test completato =====")
