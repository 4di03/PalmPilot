#!/usr/bin/env python3
"""
Check interest across all 8 landing pages.

Shows email signups per niche from Formspree, ranked by count.

Usage:
  python check_interest.py

Set FORMSPREE_API_KEY and FORMSPREE_FORM_ID in outreach/reddit/.env
"""

import os
import sys
import requests
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from dotenv import load_dotenv

# Load .env from the reddit folder (where credentials live)
env_path = Path(__file__).parent / "reddit" / ".env"
load_dotenv(dotenv_path=env_path)

FORMSPREE_API_KEY = os.getenv("FORMSPREE_API_KEY")
FORMSPREE_FORM_ID = os.getenv("FORMSPREE_FORM_ID")

NICHE_LABELS = {
    "food-truck":        "Food Truck Permits     ",
    "nonprofit-grants":  "Nonprofit Grants       ",
    "machine-shop":      "Machine Shop Maint.    ",
    "gun-shop":          "Gun Shop / FFL         ",
    "cleaning-business": "Cleaning Business      ",
    "ada-compliance":    "ADA/WCAG Compliance    ",
    "auto-detailing":    "Auto Detailing         ",
    "martial-arts":      "Martial Arts Dojo      ",
    "unknown":           "Unknown                ",
}


def fetch_submissions():
    if not FORMSPREE_API_KEY or not FORMSPREE_FORM_ID:
        print("Error: FORMSPREE_API_KEY and FORMSPREE_FORM_ID not set.")
        print(f"Edit {env_path} with your Formspree credentials.")
        sys.exit(1)

    headers = {"Authorization": f"Bearer {FORMSPREE_API_KEY}"}
    url = f"https://formspree.io/api/0/forms/{FORMSPREE_FORM_ID}/submissions"

    all_submissions = []
    page = 0

    while True:
        resp = requests.get(url, headers=headers, params={"page": page})
        if resp.status_code == 401:
            print("Error: Invalid Formspree API key.")
            sys.exit(1)
        resp.raise_for_status()

        data = resp.json()
        submissions = data.get("submissions", [])
        if not submissions:
            break

        all_submissions.extend(submissions)
        if len(submissions) < data.get("per_page", 100):
            break
        page += 1

    return all_submissions


def summarize(submissions):
    counts = defaultdict(int)
    recents = defaultdict(list)

    for s in submissions:
        niche = s.get("data", {}).get("niche", "unknown")
        counts[niche] += 1
        submitted_at = s.get("submittedAt", "")
        if submitted_at:
            recents[niche].append(submitted_at)

    return counts, recents


def print_report(counts, recents):
    total = sum(counts.values())
    max_count = max(counts.values(), default=1)

    print(f"\n{'━'*52}")
    print(f"  Interest Report — {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print(f"  Total signups: {total}")
    print(f"{'━'*52}\n")

    sorted_niches = sorted(counts.items(), key=lambda x: x[1], reverse=True)

    for niche, count in sorted_niches:
        label = NICHE_LABELS.get(niche, f"{niche:<23}")
        bar_len = round((count / max_count) * 20) if max_count > 0 else 0
        bar = "█" * bar_len
        latest = ""
        if recents[niche]:
            latest = f"  last: {sorted(recents[niche])[-1][:10]}"
        print(f"  {label}  {count:>3}  {bar}{latest}")

    print(f"\n{'━'*52}")

    if total == 0:
        print("\n  No signups yet. Deploy the pages and start posting.\n")
    elif total < 10:
        print("\n  Keep going — need more data to pick a winner.\n")
    else:
        top_niche, top_count = sorted_niches[0]
        top_label = NICHE_LABELS.get(top_niche, top_niche).strip()
        print(f"\n  Leading niche: {top_label} ({top_count} signups)\n")


def main():
    print("Fetching submissions from Formspree...")
    submissions = fetch_submissions()
    counts, recents = summarize(submissions)
    print_report(counts, recents)


if __name__ == "__main__":
    main()
