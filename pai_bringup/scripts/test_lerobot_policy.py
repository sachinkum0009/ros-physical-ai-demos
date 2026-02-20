from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
import torch

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# model_id = "lerobot/smolvla_base"
model_id = "sachinkum0009/smolvla_toothbrush4"
policy = SmolVLAPolicy.from_pretrained(model_id).to(device)

policy.eval()

print("policy loadded successfully")
# print(policy.)